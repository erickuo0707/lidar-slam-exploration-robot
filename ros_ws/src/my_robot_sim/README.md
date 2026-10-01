# 雲端模擬（ROS Noetic + Gazebo Classic）

這個套件讓專題的 `smart_explorer.py` 在沒有 Tinker Board、RPLIDAR 和 GPIO 馬達時仍可測試。Gazebo 雷射取代 `/dev/ttyUSB0` 的 RPLIDAR，Gazebo 差速驅動取代 `motor_driver_fixed.py`；**模擬時不要啟動實體雷達或 GPIO 馬達程式**。

車體和辦公室場景是依照片做的簡化近似模型，尺寸不是實車量測值；適合驗證 `/scan` → Hector SLAM → `/map`、`/slam_out_pose` → frontier 探索 → `/cmd_vel` 流程，不代表真車性能或場地的精確重建。

## 啟動

在已安裝 ROS Noetic、Gazebo Classic、`gazebo_ros_pkgs` 和 Hector SLAM 的環境中：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch my_robot_sim cloud_sim.launch
```

## GitHub Codespaces 雲端 headless 執行

本 repo 的 `.devcontainer/devcontainer.json` 會準備 ROS Noetic、Gazebo Classic 與 Hector SLAM。Codespaces 終端機在 repo 根目錄執行：

```bash
source /opt/ros/noetic/setup.bash
cd 程式碼/ros_ws
catkin_make
source devel/setup.bash
roslaunch my_robot_sim cloud_sim.launch gui:=false
```

headless 模式不開 Gazebo 視窗，但仍會模擬雷射、差速移動、SLAM 與自動探索。模擬中不要啟動實體雷達或 GPIO 馬達程式。

Gazebo 和 `smart_explorer.py` 會一起啟動。另開終端機執行 `rviz`，將 Fixed Frame 設成 `map`，加入 Map (`/map`) 和 LaserScan (`/scan`) 顯示即可觀察建圖。若要只建圖、不自動移動，可加 `start_explorer:=false`。

`ai_observer.py` 預設不啟動；如要另行測試，可設 `start_ai_observer:=true`，並確認 OpenCV/NumPy 可用及 ONNX 模型輸出類別順序符合 `classes.json`。它不是建圖所需節點。CNN 訓練 notebook 為 `../my_robot/models/train_lidar_cnn_colab.ipynb`。
