#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rodong_teleop.py  —  RODONG13 keyboard control node
Topics published:
  /rodong/cmd         std_msgs/String   "INIT" | "STOP"
  /rodong/manual_goal geometry_msgs/Point  x, y (cm), z=0
"""

import sys
import tty
import termios
import threading
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

BANNER = """
============================================================
  RODONG Teleop  (RODONG13)
------------------------------------------------------------
  i        : INIT  — 자율주행 시작
  s        : STOP  — 전체 정지
  m        : MANUAL — 좌표 입력 모드 진입
  q / Ctrl+C : 종료
============================================================
"""

MANUAL_HELP = """
[MANUAL MODE]  목표 좌표를 입력하세요 (현재 위치 기준 상대좌표, cm 단위)
  예)  100 0     →  정면 100cm
       0 50      →  왼쪽 50cm
      -80 0      →  후방 80cm
  'b' + Enter   →  MANUAL 모드 취소 (자율주행 복귀)
"""


def get_key():
    """터미널에서 단일 키 읽기 (non-blocking)"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def get_line():
    """MANUAL 모드: 터미널을 일반 모드로 복원 후 한 줄 읽기"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    termios.tcsetattr(fd, termios.TCSADRAIN, old)  # 이미 normal이지만 보험
    return sys.stdin.readline().strip()


class TeleopNode:
    def __init__(self):
        rospy.init_node('rodong_teleop', anonymous=False)
        self.pub_cmd  = rospy.Publisher('/rodong/cmd',         String, queue_size=1)
        self.pub_goal = rospy.Publisher('/rodong/manual_goal', Point,  queue_size=1)
        self.manual_mode = False

    def send_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub_cmd.publish(msg)
        rospy.loginfo('[Teleop] CMD → %s', cmd)

    def send_goal(self, x_cm, y_cm):
        msg = Point()
        msg.x = float(x_cm)
        msg.y = float(y_cm)
        msg.z = 0.0
        self.pub_goal.publish(msg)
        rospy.loginfo('[Teleop] GOAL → x=%.1f cm  y=%.1f cm', x_cm, y_cm)

    def enter_manual(self):
        self.manual_mode = True
        self.send_cmd('MANUAL')
        print(MANUAL_HELP)
        while not rospy.is_shutdown() and self.manual_mode:
            try:
                line = get_line()
            except (EOFError, KeyboardInterrupt):
                break

            if line.lower() == 'b':
                print('[MANUAL] 취소 → 자율주행 복귀')
                self.manual_mode = False
                self.send_cmd('INIT')
                break

            parts = line.split()
            if len(parts) == 2:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    self.send_goal(x, y)
                    print('[MANUAL] 목표 전송: x={:.1f}  y={:.1f} cm'.format(x, y))
                    print("  다음 좌표 입력 (또는 'b' 로 종료):")
                except ValueError:
                    print('[MANUAL] 숫자를 입력하세요. 예) 100 0')
            else:
                print("[MANUAL] 형식 오류. 예) 100 0  /  'b' 로 종료")

    def run(self):
        print(BANNER)
        print('노드 준비 완료. 키를 누르세요...')
        while not rospy.is_shutdown():
            key = get_key()

            if key in ('\x03', 'q'):          # Ctrl+C or q
                print('\n[Teleop] 종료합니다.')
                self.send_cmd('STOP')
                break

            elif key == 'i':
                self.send_cmd('INIT')
                print('[Teleop] 자율주행 시작')

            elif key == 's':
                self.send_cmd('STOP')
                print('[Teleop] 정지')

            elif key == 'm':
                print('[Teleop] MANUAL 모드 진입')
                self.enter_manual()

            else:
                pass  # 기타 키 무시


if __name__ == '__main__':
    try:
        TeleopNode().run()
    except rospy.ROSInterruptException:
        pass
