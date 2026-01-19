# ROS2 Assignment – e0509_control

본 프로젝트는 ROS2 환경에서 Python 기반 노드를 구현하는 과제로
ROS2 패키지 구조를 준수하여 `colcon build` 및 `ros2 run` 명령으로
정상 실행이 가능하도록 구성하였다.

## 1. 실행 매뉴얼
### 1) Project Structure
```
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
```

### 2) Environment

- OS: Ubuntu 20.04 / 22.04
- ROS2: Humble
- Python: 3.10.12
- Build Tool: colcon
- Package Type: ament_python

### 3) External Dependency (Doosan Robot)

본 과제는 Doosan 로봇 암 ROS2 패키지를 기반으로 동작한다.  
해당 패키지는 본 레포지토리에 포함하지 않으며,
아래와 같이 별도로 다운로드하여 사용한다.

#### Doosan Robot Package 다운로드 및 실행

```bash
cd ros2_ws/src
git clone https://github.com/doosan-robotics/doosan-robot2.git
cd ros2_ws
colcon build
source install/setup.bash
ros2 run e0509_control my_node
```

## 2. 작동로직

### 1) 작동로직 상세

```
1. ROS2 노드가 초기화되며 e0509_control 패키지의 메인 노드가 실행
2. 로봇 암 제어를 위한 ROS2 통신 인터페이스가 설정
3. 사용자 입력에 따라 로봇 제어 명령 생성
4. 생성된 제어 명령은 ROS2 실행 환경을 통해 로봇 제어 시스템으로 전달
5. 로봇의 동작 상태를 주기적으로 확인하며 실행 결과를 로그로 출력
6. 노드 졸료 시 ROS2 리소를 정리하고 정상적으로 종료

위 과정은 ROS2기반 실행 구조를 따르며 노드는 단일 프로세스 환경에서 동작하도록 설계하였다.
```
