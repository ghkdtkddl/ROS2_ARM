#!/usr/bin/env python3
"""
gui_app.py

PyQt5 기반 Doosan E0509 MoveIt Control GUI
- 목표 좌표 입력
- 절대 / 상대 좌표 선택
- 속도 / 가속도 설정
- 실행 상태 / 로봇 상태 모니터링
"""

import sys
import time

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout,
    QLabel, QPushButton, QLineEdit, QTextEdit,
    QComboBox, QTableWidget
)
from PyQt5.QtCore import QThread, pyqtSignal

import rclpy
# from moveit_controller import MoveItController    
from e0509_control.moveit_controller import MoveItController

# ===============================
# 로봇 제어 스레드
# ===============================
class RobotWorker(QThread):
    log_signal = pyqtSignal(str)
    state_signal = pyqtSignal(str)

    def __init__(self, coord_type, targets, vel, acc):
        super().__init__()
        self.coord_type = coord_type
        self.targets = targets
        self.vel = vel
        self.acc = acc

    def run(self):
        rclpy.init()
        node = MoveItController()

        node.max_velocity = self.vel
        node.max_acceleration = self.acc

        self.log_signal.emit("로봇 연결됨")
        self.log_signal.emit("상태: 실행 중")

        # 좌표 기준 처리
        if self.coord_type == 0:
            targets = self.targets
            self.log_signal.emit("좌표 기준: 절대 좌표")
        else:
            self.log_signal.emit("좌표 기준: 상대 좌표")
            targets = []
            bx, by, bz = 0.0, 0.0, 0.0
            for dx, dy, dz in self.targets:
                bx += dx
                by += dy
                bz += dz
                targets.append((bx, by, bz))

        # ===== 순차 실행 =====
        for x, y, z in targets:
            self.log_signal.emit(f"Planning to ({x:.2f}, {y:.2f}, {z:.2f})")
            traj = node.plan_to_pose(x, y, z)
            if traj:
                node.execute_trajectory(traj)
                self.log_signal.emit("Execution finished")
            else:
                self.log_signal.emit("Planning failed")

            # 상태 업데이트
            joint_text = node.get_joint_state_text()
            ee_pos = node.get_ee_position_estimate()

            if ee_pos:
                ex, ey, ez = ee_pos
                state_text = (
                    f"[Joint Angles]\n{joint_text}\n\n"
                    f"[EE Position (base)]\n"
                    f"x={ex:.2f}, y={ey:.2f}, z={ez:.2f}"
                )
            else:
                state_text = f"[Joint Angles]\n{joint_text}"

            self.state_signal.emit(state_text)
            time.sleep(0.2)

        node.destroy_node()
        rclpy.shutdown()

        self.log_signal.emit("상태: 정지")
        self.log_signal.emit("로봇 동작 종료")


# ===============================
# 메인 GUI
# ===============================
class MoveItGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Doosan E0509 MoveIt Control GUI")
        self.resize(700, 650)

        layout = QVBoxLayout()

        # 좌표 기준
        self.coord_box = QComboBox()
        self.coord_box.addItems(["절대 좌표", "상대 좌표"])
        layout.addWidget(QLabel("좌표 기준"))
        layout.addWidget(self.coord_box)

        # 목표 좌표 테이블
        self.table = QTableWidget(3, 3)
        self.table.setHorizontalHeaderLabels(["X", "Y", "Z"])
        layout.addWidget(QLabel("목표 좌표 (3개)"))
        layout.addWidget(self.table)

        # 속도 / 가속도
        self.vel_edit = QLineEdit("0.2")
        self.acc_edit = QLineEdit("0.2")

        layout.addWidget(QLabel("최대 속도 (0~1)"))
        layout.addWidget(self.vel_edit)
        layout.addWidget(QLabel("최대 가속도 (0~1)"))
        layout.addWidget(self.acc_edit)

        # 실행 버튼
        self.run_btn = QPushButton("실행")
        self.run_btn.clicked.connect(self.run_robot)
        layout.addWidget(self.run_btn)

        # 상태 표시
        self.state_box = QTextEdit()
        self.state_box.setReadOnly(True)
        layout.addWidget(QLabel("로봇 상태"))
        layout.addWidget(self.state_box)

        # 로그
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        layout.addWidget(QLabel("실시간 로그"))
        layout.addWidget(self.log_box)

        self.setLayout(layout)

    def run_robot(self):
        targets = []
        for row in range(self.table.rowCount()):
            try:
                x = float(self.table.item(row, 0).text())
                y = float(self.table.item(row, 1).text())
                z = float(self.table.item(row, 2).text())
                targets.append((x, y, z))
            except:
                continue

        vel = float(self.vel_edit.text())
        acc = float(self.acc_edit.text())
        coord_type = self.coord_box.currentIndex()

        self.worker = RobotWorker(coord_type, targets, vel, acc)
        self.worker.log_signal.connect(self.log_box.append)
        self.worker.state_signal.connect(self.state_box.setText)
        self.worker.start()


def main():
    app = QApplication(sys.argv)
    gui = MoveItGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
