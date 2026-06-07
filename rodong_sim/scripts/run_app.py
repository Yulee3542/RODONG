#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_app.py — patch 디렉터리의 애플리케이션 노드를 단일 소스로 실행하는 런처.

rodong_main.py / vfh_planner.py 는 patch(20260602) 폴더에서 형제 모듈
(rodong_config 등)을 import 하므로, 그 폴더를 sys.path 에 넣고 원본 파일을
__main__ 으로 실행한다. 시뮬용으로 코드를 복사하지 않아 원본과 항상 일치한다.

  rosrun rodong_sim run_app.py <app_dir> <module_name>
  예) rosrun rodong_sim run_app.py /path/to/patch(20260602) rodong_main
roslaunch 가 뒤에 붙이는 __name:= / __log:= 인자는 그대로 보존해 rospy 에 전달한다.
"""

import os
import sys
import runpy

if len(sys.argv) < 3:
    sys.stderr.write('usage: run_app.py <app_dir> <module_name> [ros args...]\n')
    sys.exit(2)

app_dir = sys.argv[1]
module  = sys.argv[2]
ros_args = sys.argv[3:]          # roslaunch 가 추가한 __name:=, __log:= 등

target = os.path.join(app_dir, module + '.py')
if not os.path.isfile(target):
    sys.stderr.write('run_app.py: not found: %s\n' % target)
    sys.exit(2)

sys.path.insert(0, app_dir)
# 원본 파일이 보는 argv: [모듈, <ros remap args>]
sys.argv = [target] + ros_args

runpy.run_path(target, run_name='__main__')
