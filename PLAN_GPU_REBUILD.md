# RTX 5080 向け GPU 環境再構築プラン（Ubuntu 22.04 + Python 3.10）

## 背景
- 現行コンテナは Ubuntu 20.04 + ROS Noetic + Python 3.8。PyTorch 1.12/cu113 は sm_120 (RTX 5080) 非対応。
- sm_120 対応は CUDA 12.8 (cu128) ビルドの torch 2.8+ 以降かつ Python 3.10+ のみ。公式ホイールは 3.8/20.04 では入手不可。
- GPU を活かすには環境を作り直すか、対応 GPU (sm_89 等) に交換する必要がある。

## 方針
- 新規 Docker イメージを用意（既存 20.04 を壊さない）。
- ベース OS: Ubuntu 22.04。
- Python: 3.10 系。
- ROS: Noetic をソースビルドして Python 3.10 対応にする（公式外、依存は手動対応）。
- venv に sm_120 対応の PyTorch (cu128/torch 2.8+) を導入し、`policy_device: cuda` で動かす。
- ai_module/system を Python 3.10 + 新 Noetic で再ビルド。
- VLM サーバ (GroundingDINO/BLIP2/SAM/YOLO) も同コンテナで GPU 起動。

## 手順（概要）
1. **ベース準備**  
   `apt-get install -y python3.10 python3.10-dev python3.10-venv git build-essential cmake` などを導入。
2. **Noetic ソース取得 & 依存**  
   `rosinstall_generator desktop --rosdistro noetic --tar > noetic-desktop.rosinstall`  
   `vcs import --input noetic-desktop.rosinstall src`  
   `rosdep init && rosdep update` 後、`rosdep install --from-paths src --ignore-src -y --rosdistro noetic`（不足は手動追加）。
3. **Noetic を Python 3.10 でビルド**  
   `PYTHON_EXECUTABLE=/usr/bin/python3.10` を指定して  
   `./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}`  
   完了後 `source ~/ros_catkin_ws/install_isolated/setup.bash`。
4. **PyTorch (sm_120 対応) 導入**  
   `python3.10 -m venv ~/uiap-venv && source ~/uiap-venv/bin/activate`  
   `pip install --upgrade pip`  
   sm_120 対応の torch/cu128 (torch 2.8+ など) を導入。ホイールが無ければ `TORCH_CUDA_ARCH_LIST="12.0"` でソースビルドを検討。
5. **CMU-VLA-Challenge 再ビルド**  
   `ai_module` で `rm -rf build devel` → `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3.10`  
   必要なら `system/unity` も同様に 3.10 でビルド。
6. **uiap-ogn インストール**  
   venv 内で `pip install -e . --no-build-isolation` を実行し、`frontier_exploration` 等の git 依存・VLM 依存も導入。
7. **設定と起動**  
   `params.yaml` を絶対パス & `policy_device: "cuda"` に更新。  
   VLM サーバ用の環境変数/ポートを確認。  
   新 Noetic + venv を `source` して `roslaunch uiap_ogn_ros uiap_ogn.launch start_vlm_servers:=true` → `rostopic pub/echo` で確認。

## リスクと注意
- Noetic の 3.10 ビルドは公式外。Qt/rviz などの依存でビルド失敗の可能性あり。
- 評価環境は 20.04+Noetic 前提なので互換性は自前確認が必要。
- sm_120 対応ホイールが無ければ torch のソースビルドが必要（時間・ディスクを消費）。
- 安全のため、新コンテナで作業し、現行環境は温存する。

## 代替案
- 対応 GPU (sm_89 の RTX 4080S 等) に差し替えれば、20.04+py3.8+cu121 公式ホイールでそのまま動く。
- VLM だけ外部 GPU ホストで動かし、ROS/Noetic は現状維持（ただし PointNav/ZoeDepth は CPU で重い）。
