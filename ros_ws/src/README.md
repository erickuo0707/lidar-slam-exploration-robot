# ROS 工作區原始碼

這裡是 catkin 工作區的 `src/`，包含兩個 ROS 套件。

## my_robot：專題主程式

| 路徑 | 內容 |
| --- | --- |
| `launch/hector_stable.launch` | Hector SLAM 建圖設定（地圖解析度 5 cm、三層多解析度、雷射 0.15–5.5 m） |
| `scripts/smart_explorer.py` | 自主探索：frontier 選目標、扇區避障、卡住脫困、以 SLAM 位姿回授的航向比例控制 |
| `scripts/motor_driver_fixed.py` | 馬達驅動：把 `/cmd_vel` 換算成左右輪伺服馬達脈衝，含平滑與 1 秒逾時停車 |
| `scripts/data_collector.py` | 把雷射掃描畫成 128×128 影像，收集場景資料 |
| `scripts/ai_observer.py` | 在車上以 ONNX 模型即時判斷所在場景（走廊／轉角／右轉角／房間） |
| `scripts/lidar_brain.onnx`、`classes.json` | 專題期間在車上使用的初版模型與類別標籤 |
| `maps/` | 實測建立的地圖 |
| `dataset/` | LiDAR 場景影像，4 類 × 100 張 |
| `models/` | 重新訓練的 CNN 模型、Colab 訓練筆記本與結果（測試準確率 80%） |

## my_robot_sim：Gazebo 模擬

沒有實體車時，用 Gazebo 的雷射與差速驅動取代 RPLIDAR 和馬達，執行「掃描 → 建圖 → 探索」流程。包含簡化的車體模型（`urdf/`）、辦公室場景（`worlds/`）與啟動檔（`launch/cloud_sim.launch`），說明見 [my_robot_sim/README.md](./my_robot_sim/README.md)。

## 建置

```bash
cd ros_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

雷達驅動請另外安裝官方套件 `ros-noetic-rplidar-ros`。
