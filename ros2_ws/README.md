\# ROS2 Assignment – e0509\_control



본 프로젝트는 ROS2 환경에서 Python 기반 노드를 구현하는 과제로

ROS2 패키지 구조를 준수하여 `colcon build` 및 `ros2 run` 명령으로

정상 실행이 가능하도록 구성하였다.



\## 1. 실행 매뉴얼

\### 1) Project Structure

```

ros2\_ws/

├─ src/

│ └─ e0509\_control/

│ ├─ package.xml

│ ├─ setup.py

│ ├─ setup.cfg

│ ├─ resource/

│ │ └─ e0509\_control

│ └─ e0509\_control/

│ ├─ init.py

│ └─ my\_node.py

├─ ROS2를\_이용한\_로봇암.pptx

├─ README.md

└─ requirements.txt

```



\### 2) Environment



\- OS: Ubuntu 20.04 / 22.04

\- ROS2: Humble

\- Python: 3.10.12

\- Build Tool: colcon

\- Package Type: ament\_python



\### 3) External Dependency (Doosan Robot)



본 과제는 Doosan 로봇 암 ROS2 패키지를 기반으로 동작한다.  



