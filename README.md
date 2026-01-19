# ROS2 Assignment – e0509_control

본 프로젝트는 ROS2 환경에서 Python 기반 노드를 구현하는 과제로
ROS2 패키지 구조를 준수하여 `colcon build` 및 `ros2 run` 명령으로
정상 실행이 가능하도록 구성하였다.

## 1. Project Structure
ros2_ws/
├─ src/
│ └─ e0509_control/
│ ├─ package.xml
│ ├─ setup.py
│ ├─ setup.cfg
│ ├─ resource/
│ │ └─ e0509_control
│ └─ e0509_control/
│ ├─ init.py
│ └─ my_node.py
├─ ROS2를_이용한_로봇암.pptx
├─ README.md
└─ requirements.txt

## 2. Environment

- OS: Ubuntu 20.04 / 22.04
- ROS2: Humble
- Python: 3.10.12
- Build Tool: colcon
- Package Type: ament_python

## 3. External Dependency (Doosan Robot)

본 과제는 Doosan 로봇 암 ROS2 패키지를 기반으로 동작한다.  
해당 패키지는 본 레포지토리에 포함하지 않으며,
아래와 같이 별도로 다운로드하여 사용한다.

### Doosan Robot Package 다운로드

```bash
cd ros2_ws/src
git clone https://github.com/doosan-robotics/doosan-robot2.git


