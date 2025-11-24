# uiap-ogn を CMU-VLA-Challenge に ROS 統合するための検討メモ

本メモは、`uiap-ogn` を ROS（Noetic）環境の CMU-VLA-Challenge AI モジュールとして動かすための方針・手順・リスク確認をまとめたものです。  
対象コード: `/Users/morigakuto/uiap-ogn`（Hydra/Habitat/Spot 用 Python パイプライン）と `/Users/morigakuto/CMU-VLA-Challenge`（ROS catkin + Unity/Matterport システム）。

## 現状整理
- CMU-VLA-Challenge: `/challenge_question` 受信、出力は `/way_point_with_heading` (Pose2D), `/selected_object_marker` (Marker), `/numerical_response` (Int32)。ダミーモデルは C++ ノード。
- uiap-ogn: `RealityITMPolicyV2`（frontier + value map + PointNav）を Spot 実機向けに実装。入力は `nav_depth`・`obstacle_map_depths`・`value_map_rgbd`・`object_map_rgbd`・`robot_xy`・`robot_heading`。VLM サーバ群（GroundingDINO / BLIP2-ITM / MobileSAM / YOLOv7）を Flask で常駐させる設計。
- 既存 ROS データ: 360 カメラ `/camera/image`、LiDAR `/registered_scan` 等。GT 物体は `/object_markers` として利用可能。

## ROS 化の技術方針
1) **依存環境を Docker 内に統合**  
   - `docker/Dockerfile` を拡張し、Python3.9 + PyTorch1.12 系と uiap-ogn 依存 (`frontier_exploration`, `mobile_sam`, `depth_camera_filtering`, `groundingdino`, `yolov7`, `salesforce-lavis`, `open3d` など) を `pip install -e` で導入。  
   - `GroundingDINO`/`yolov7`/`MobileSAM` のリポジトリクローンと重量ファイルを `ai_module` 配下に配置。`scripts/launch_vlm_servers.sh` が tmux なしでも動くよう `nohup` 版スクリプトを用意。

2) **catkin パッケージ新設**  
   - `ai_module/src/uiap_ogn_ros` を作り、`rospy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`, `message_filters`, `tf2_ros` を依存に設定。  
   - `setup.py`/`PYTHONPATH` で `/home/$USER/CMU-VLA-Challenge/uiap-ogn` を読み込む。

3) **観測変換ノード**（`uiap_ogn_ros/ros_observation_builder.py`）  
   - `/camera/image` + `/registered_scan` を同期。LiDAR点群を前方視野へ射影し、`nav_depth`（正規化深度）を生成。  
   - `obstacle_map_depths`: (深度画像, tf_camera_to_episodic, min/max depth, fx/fy, topdown_fov) のリストを構築。カメラ→LiDAR→map の外部キャリブは付属 README（Real-Robot data）や既知の変換を使用し、`tf2` 経由で矩形行列を生成。  
   - `value_map_rgbd` / `object_map_rgbd`: RGB + 同じ深度 (または推定深度) を渡す。  
   - `robot_xy`/`robot_heading`: `/state_estimation` から map 座標を取得し、uiap-ogn のエピソード座標系（x 前方、y 左）に合わせる。初期姿勢を `tf_episodic_to_global` として保持。
   - オプション: `/object_markers` を利用して GT セマンティクスを `ObjectPointCloudMap` に直接投入するショートカットを実装（検出失敗時のフォールバック）。

4) **ポリシーラッパノード**（`uiap_ogn_ros/policy_node.py`）  
   - `RealityITMPolicyV2` を内包し、`_cache_observations` 相当を ROS からの観測辞書にマップ。  
   - ループで `policy.get_action` を呼び、`_policy_info` から現在のゴール/フロンティアを取り出して `geometry_msgs/Pose2D` を `/way_point_with_heading` へ送信。`value_map` などのデバッグ画像を RViz Image/Marker にオプション配信。  
   - 数値質問は暫定で `/object_markers` を数え上げて `/numerical_response` に返す。  
   - 物体参照時は VLM 検出結果（または GT）を `visualization_msgs/Marker` に整形し `/selected_object_marker` へ。

5) **クエリ制御ノード**（`uiap_ogn_ros/query_router.py`）  
   - `/challenge_question` を購読し、質問タイプ（数値 / find / 経路）を簡易パーサで判定。ターゲット名をポリシーに設定しリセット。  
   - 10 分タイマーで探索を打ち切り、到達 or タイムアウトで停止。  
   - 全ノードを `uiap_ogn_ros/launch/uiap_ogn.launch` にまとめ、`launch_module.sh` から起動するように差し替え。

## 実装ステップ
- S1: Dockerfile に Python 環境と uiap-ogn 依存を追加、`pip install -e /home/$USER/CMU-VLA-Challenge/uiap-ogn`。モデル重量を `ai_module/data` に配置。  
- S2: `uiap_ogn_ros` パッケージを生成し、`CMakeLists.txt`/`package.xml` 設定。  
- S3: 観測変換ノード実装（点群射影→深度、TF 生成、オプションで GT semantics 取り込み）。  
- S4: ポリシーラッパ＋クエリ制御ノード実装、トピック I/O を Challenge 仕様に合わせる。  
- S5: `launch_module.sh` を新ノード起動に更新。  
- S6: 検証（下記チェック3回）と RViz での可視化確認。

## 3 回の計画チェック
1. **データ整合性チェック**  
   - 座標系: map と episodic の軸反転がないか、`/state_estimation` と `tf` を比較。  
   - 深度生成: LiDAR 射影の FOV/fx/fy が `get_point_cloud` 前提（m 単位・正規化）と一致するか bag で確認。
2. **依存・ビルドチェック**  
   - Docker 内で `catkin_make` + `pip install -e uiap-ogn` が衝突しないか。  
   - CUDA/torch バージョンが GPU と一致するか、VLM ロード時間と GPU メモリを計測。
3. **動作・評価チェック**  
   - `/way_point_with_heading` 出力が `traversable_area` 外に出ないか RViz で検証。  
   - 質問タイプごとのフォールバック（検出失敗→GT、タイムアウト→停止）を確認し、10 分制限内で走ることを実測。

以上を順に実施すれば、uiap-ogn のフロンティア探索＋価値マップ計画を ROS システムに統合し、Challenge の入出力仕様に適合させることができます。
