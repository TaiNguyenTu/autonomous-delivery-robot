# autonomous-delivery-robot
GPS + IMU outdoor navigation for wheeled robot using LOS Guidance and PID Heading Controller (Jetson TX2 + STM32 + ROS)
---

## 🧠 Key Features

- ✅ **Line-of-Sight (LOS)** guidance algorithm with cross-track/along-track error
- ✅ **Heading controller (PID)** with real-time tuning via `dynamic_reconfigure`
- ✅ **Sensor fusion** from:
  - GPS RTK (Here+ v2)
  - IMU BNO055
  - Wheel encoders
- ✅ **CAN communication** (UART-CAN) to STM32F4 motor board
- ✅ Full **data logging** for performance evaluation (CTE, heading error, wheel speed...)

---

## 🛠 How to Run

```bash
cd mybot_ws
catkin_make  --pkg utils
catkin_make
source devel/setup.bash
./run_master.sh
