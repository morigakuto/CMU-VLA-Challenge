# uiap-ogn を CMU-VLA-Challenge に統合する新方針メモ（ROSノード薄ラップ）

目的: uiap-ogn 本体はそのまま（Hydra/Habitat向けPythonライブラリ）、ROS側に最小のラッパーパッケージを追加して、CMU VLA シミュレータの公開トピックだけで動かす。Dockerは提出しない個人実験では不要（ただし提出時は別途コンテナ化を想定）。

## 現状と制約
- CMU-VLA: 入力 `/challenge_question`、出力 `/way_point_with_heading` (Pose2D)・`/selected_object_marker`・`/numerical_response`。センサは `/camera/image`, `/registered_scan`（または `/sensor_scan`）, `/state_estimation`, `/traversable_area`, `/object_markers` 等。
- uiap-ogn: `RealityITMPolicyV2` が要求する観測辞書（`nav_depth`, `obstacle_map_depths`, `value_map_rgbd`, `object_map_rgbd`, `robot_xy`, `robot_heading`）。VLMバックエンドは Flask サーバ（GroundingDINO/BLIP2-ITM/MobileSAM/YOLOv7）を叩く。
- 課題: ROSメッセージを uiap-ogn 期待の numpy/Tensor 形式に変換し、ポリシーの出力を ROS トピックに載せるブリッジが未実装。

## 推奨ディレクトリ構成（catkin パッケージのみ追加）
```
ai_module/
  src/uiap_ogn_ros/
    package.xml, CMakeLists.txt
    launch/uiap_ogn.launch          # 観測ブリッジ＋ポリシーノード（必要ならVLM起動）
    config/params.yaml              # カメラ/LiDAR内外部、トピック名、FOV、タイマー
    scripts/launch_vlm_servers_nohup.sh  # Flaskサーバ起動（任意）
    src/
      observation_bridge.py         # ROS→uiap観測辞書
      policy_node.py                # RealityITMPolicyV2 呼び出し→Pose2D/Marker/Int32 出力
      question_router.py            # challenge_question パース・モード切替
      gt_semantics_adapter.py       # /object_markers を ObjectMap に反映（任意）
```
uiap-ogn 本体は `/Users/morigakuto/uiap-ogn` を `pip install -e` で参照し、変更しない。

## 実装ステップ（非Docker前提）
1. **環境準備**  
   - ROS Noetic + Python 3.9 環境を用意。`pip install -e /Users/morigakuto/uiap-ogn`。Torch 1.12 系と依存を README 通りに導入。VLM 重量（MobileSAM, GroundingDINO, YOLOv7, pointnav_weights 等）を `ai_module/data` に配置。  
   - ROS マスターをシミュレータ Docker に向ける: `export ROS_MASTER_URI=http://<sim_host>:11311`、`export ROS_IP=<host_ip>`。
2. **catkin パッケージ作成**  
   - `ai_module/src/uiap_ogn_ros` を `catkin_create_pkg` で生成し、依存に `rospy sensor_msgs nav_msgs geometry_msgs visualization_msgs std_msgs message_filters tf2_ros` を追加。`catkin_make` でビルド。
3. **観測ブリッジ実装（observation_bridge.py）**  
   - `/state_estimation` から (x,y,heading) を取得。座標系を uiap-ogn の期待（x前方, y左）に揃える。  
   - `/registered_scan` を射影して `nav_depth` を作成。FOV/fx/fy/外部キャリブから TF を組み、`obstacle_map_depths` を構築（深度, tf, min/max, fx/fy, topdown_fov のタプル配列）。  
   - `/camera/image` から RGB を取得し、対応する深度・TF を付けて `value_map_rgbd` / `object_map_rgbd` を生成。必要に応じ `/object_markers` から GT を `gt_semantics_adapter` 経由で ObjectMap に投入。
4. **ポリシーノード実装（policy_node.py）**  
   - `RealityITMPolicyV2` を初期化し、観測辞書を渡して `get_action`。取得したフロンティア/ゴールを Pose2D に変換し `/way_point_with_heading` へ出力。  
   - find系は Marker を `/selected_object_marker`、数値系は `/object_markers` カウント等で `/numerical_response` に送る。  
   - デバッグ用に value_map/obstacle_map の Image/Marker を任意で配信。
5. **質問ルータ（question_router.py）**  
   - `/challenge_question` を購読し、先頭語で簡易分類（how many / find / その他）。ターゲット名をポリシーに設定しリセット。10分タイマーと停止処理を持たせる。  
6. **起動統合**  
   - `uiap_ogn.launch` で観測ブリッジ＋ポリシーノード（＋必要なら VLM サーバ起動スクリプト）をまとめる。`launch_module.sh` を `roslaunch uiap_ogn_ros uiap_ogn.launch` に差し替え。  
7. **動作確認**  
   - 最初はダミー観測で policy が動くことを確認→LiDAR射影→本番トピックに切替。RViz で Pose2D と Marker が妥当か確認。

## 3 回の計画チェック（リスク洗い出しと対策）
1. **データ整合性**  
   - 座標系/符号ずれ: `/state_estimation` の y 方向が uiap-ogn と逆なら符号反転する。TF 行列を rviz で可視化して前方・左が一致するか確認。  
   - 深度正規化: LiDAR射影はメートル→[0,1] に正規化し、fx/fy/FOV が `get_point_cloud` 前提と一致するか bag リプレイで確認。
2. **依存とリソース**  
   - Python/ROS 共存: `pip install -e uiap-ogn` がシステム Python ではなく専用環境で動くことを確認。catkin と衝突しない。  
   - VLM 重量: GPU/メモリが足りない場合は GT `/object_markers` を優先利用し、VLM は後付けにする逃げ道を用意。
3. **動作・評価パス**  
   - トピック整合: `/way_point_with_heading` が `traversable_area` 外に行かないよう、発行前に最近傍の traversable に投影するガードを入れる。  
   - 質問フォールバック: 検出失敗時は GT セマンティクスで回答、タイマー超過時は停止を確実に送る。10分制限をタイマーで強制し、未完了でも安全終了。

この構成なら「uiap-ogn 本体は手付かず」「ROS側に薄いブリッジだけ追加」で、非Dockerの個人環境でもシミュレータのトピックだけで動かせます。提出が必要になったら、同じレイアウトをベースに Dockerfile を後付けすればよいです。

## 現在の実装状況
- `ai_module/src/uiap_ogn_ros` を catkin パッケージとして追加済み。`observation_bridge.py` で `/camera/image` `/registered_scan` `/state_estimation` から uiap の観測辞書を構築し、`policy_node.py` で `RealityITMPolicyV2` を呼び出して `/way_point_with_heading` と `/selected_object_marker` `/numerical_response` に出力する。
- 質問は `question_router.py` が `/challenge_question` を簡易パースして `/uiap_ogn/object_goal` と `/uiap_ogn/query_type` に流す。`launch/uiap_ogn.launch` でブリッジ＋ポリシー＋ルータをまとめて起動。
- VLM サーバは `ai_module/src/uiap_ogn_ros/scripts/launch_vlm_servers_nohup.sh` で手元の `uiap-ogn` チェックアウトから起動する想定（ポート 12181-12184 デフォルト）。
- 起動例: `source ai_module/devel/setup.bash && roslaunch uiap_ogn_ros uiap_ogn.launch start_vlm_servers:=true`。`hfov_deg` や `camera_translation` は `config/params.yaml` で調整。
