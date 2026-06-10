#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_app.py — launcher that runs the application nodes from the patch directory as a single source.

rodong_main.py / vfh_planner.py import sibling modules (rodong_config etc.) from the
patch(20260602) folder, so this puts that folder on sys.path and runs the original file
as __main__. The simulator does not copy the code, so it always matches the original.

  rosrun rodong_sim run_app.py <app_dir> <module_name>
  e.g.) rosrun rodong_sim run_app.py /path/to/patch(20260602) rodong_main
The __name:= / __log:= args appended by roslaunch are preserved and passed to rospy.
"""

import os
import sys
import runpy

if len(sys.argv) < 3:
    sys.stderr.write('usage: run_app.py <app_dir> <module_name> [ros args...]\n')
    sys.exit(2)

app_dir = sys.argv[1]
module  = sys.argv[2]
ros_args = sys.argv[3:]          # __name:=, __log:= etc. added by roslaunch

target = os.path.join(app_dir, module + '.py')
if not os.path.isfile(target):
    sys.stderr.write('run_app.py: not found: %s\n' % target)
    sys.exit(2)

sys.path.insert(0, app_dir)
# argv as seen by the original file: [module, <ros remap args>]
sys.argv = [target] + ros_args

runpy.run_path(target, run_name='__main__')
