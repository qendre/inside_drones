# Indoor Drone Navigation Reproducibility Repo

This repository contains reproducible steps to run a PX4 SITL + MAVROS integration and to test vision-aided localization (cuVSLAM) + NVBlox mapping + YOLO pipe detection flow.  
Target platform used in experiments: **Ubuntu 22.04, ROS 2 Humble, PX4 vX.Y.Z, Gazebo**. See **ENVIRONMENT** below for exact versions.

> Quick overview:
> 1. Install dependencies.  
> 2. Launch PX4 SITL (Gazebo X500).  
> 3. Launch MAVROS and verify `/mavros/state`.  
> 4. Publish example vision poses or replay a bag.  
> 5. Run NVBlox + cuVSLAM and visualize in RViz.  
> 6. Run body-frame controller to follow pipe axis (simulated).

---

## ENVIRONMENT (must include exact versions)
- OS: Ubuntu 22.04 (Jammy)
- ROS 2: Humble Hawksbill (`ros-humble-desktop`)
- PX4: `PX4-Autopilot` | v1.17.0-alpha1 (`origin/main` branch) | commit shown by `git describe --tags --abbrev=0` |
- Gazebo: `gazeboX` (installed by PX4 setup script)
- MAVROS: `ros-humble-mavros`, `ros-humble-mavros-extras`
- NVIDIA (if Jetson): JetPack X.Y (note driver + CUDA versions)

> **Important:** record exact versions (run `ros2 --version`, `git rev-parse HEAD` for PX4 repo, `gz --version`).

---

## 🧩 Install Dependencies

### Install PX4 Autopilot + Gazebo

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh      # installs Gazebo and dependencies
echo "export PATH=$PATH:$HOME/PX4-Autopilot/Tools" >> ~/.bashrc
source ~/.bashrc
```

### Install MAVROS 2 (ROS ↔ PX4 bridge)
```sudo apt update
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

verify:
```ros2 pkg list | grep mavros
# should print: mavros, mavros_extras
```


### Install QGroundCOntrol
```cd ~
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage &
```
QGroundControl connects automatically over UDP port 14550.

## Launching the Simulation
### Step 1 – Start PX4 SITL (Gazebo X500)
```cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
Verify:
```INFO  [mavlink] partner IP: 127.0.0.1
INFO  [px4] Startup script returned successfully
pxh>
```
Keep this terminal open.

### Step 2 – Start MAVROS Bridge
open a new terminal
```source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557"
```
Expected:
```[INFO] [mavros]: FCU: Heartbeat received
[INFO] [mavros]: MAVROS connected
```

### Step 3 – Verify Telemetry
List topics:
```ros2 topic list | grep mavros
```
Check state and position:
```ros2 topic echo /mavros/state
ros2 topic echo /mavros/local_position/pose
```
Typical:
```connected: true
armed: false
guided: true
mode: AUTO.LOITER
```
This confirms MAVROS ↔ PX4 communication.

### Step 4 - QGroundControl
```./QGroundControl.AppImage &
```
The virtual drone will appear automatically and stream telemetry.



