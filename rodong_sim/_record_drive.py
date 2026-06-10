#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Temporary demo recorder: records one run of 2-obstacle avoid → ArUco approach → U-turn
→ produces 2 GIFs (FPV / top view). Delete after use.
(rodong_sim/demo_drive_fpv.gif, demo_drive_top.gif)"""
import time, math
import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
from xycar_msgs.msg import xycar_motor
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from cv_bridge import CvBridge
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow
from PIL import Image as PILImage

OUT = "/workspace/rodong_sim"
DUR = 40.0          # recording duration [s] — one pass of avoid + approach + U-turn
CAP = 0.3           # capture interval [s]
br = CvBridge()

S = {"img": None, "pose": None, "motor": (0, 0), "front": None, "yt": None, "at": None}

def cb_img(m): S["img"] = br.imgmsg_to_cv2(m, "bgr8")
def cb_mot(m): S["motor"] = (m.speed, m.angle)
def cb_son(m):
    d = list(m.data)
    S["front"] = d[3] if len(d) > 3 else None   # SONAR_FRONT
def cb_yo(m): S["yt"] = rospy.Time.now()
def cb_aruco(m): S["at"] = rospy.Time.now()
def cb_ms(m):
    if "rodong" in m.name:
        i = m.name.index("rodong"); p = m.pose[i].position; o = m.pose[i].orientation
        S["pose"] = (p.x, p.y, math.degrees(math.atan2(2*(o.w*o.z), 1-2*o.z*o.z)))

rospy.init_node("demo_rec", anonymous=True)
rospy.Subscriber("/usb_cam/image_raw", Image, cb_img, queue_size=1, buff_size=2**24)
rospy.Subscriber("/xycar_motor", xycar_motor, cb_mot, queue_size=1)
rospy.Subscriber("/xycar_ultrasonic", Int32MultiArray, cb_son, queue_size=1)
rospy.Subscriber("/rodong/yolo", Float32MultiArray, cb_yo, queue_size=1)
rospy.Subscriber("/aruco_pose", PoseStamped, cb_aruco, queue_size=1)
rospy.Subscriber("/gazebo/model_states", ModelStates, cb_ms, queue_size=1)
cmd_pub = rospy.Publisher("/rodong/cmd", String, queue_size=1)

# Reset start position: start slightly lowered at y=-0.20 → box1(red,-0.25) becomes nearly
# head-on so the car clearly avoids to the left (+y).
START_Y = -0.20
rospy.wait_for_service("/gazebo/set_model_state")
setm = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
ms = ModelState(); ms.model_name = "rodong"
ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = 0.0, START_Y, 0.10
ms.pose.orientation.w = 1.0
setm(ms)
time.sleep(1.2)

# Phase colors (BGR for FPV) — the top view maps by the phase string
COL = {"drive": (0, 255, 0), "AVOID": (0, 165, 255), "REVERSE": (0, 0, 255),
       "ArUco approach": (0, 255, 255), "U-TURN": (255, 0, 255)}

fpv_frames, samples = [], []
t0 = time.time(); last_cap = 0.0; started = False
seen_marker = False
rate = rospy.Rate(20)
while not rospy.is_shutdown() and time.time()-t0 < DUR:
    t = time.time()-t0
    if not started and t > 1.0:
        cmd_pub.publish(String(data="INIT")); started = True
    if t - last_cap >= CAP and S["img"] is not None and S["pose"] is not None:
        last_cap = t
        fr = S["img"].copy(); H, W = fr.shape[:2]
        spd, ang = S["motor"]
        p = S["pose"]; yaw_abs = abs(p[2])
        rev = spd < 0
        vis  = S["yt"] is not None and (rospy.Time.now()-S["yt"]).to_sec() < 0.5
        aruco_fresh = S["at"] is not None and (rospy.Time.now()-S["at"]).to_sec() < 1.0
        if aruco_fresh and p[0] > 3.4:
            seen_marker = True
        # Phase decision: large yaw near the marker = U-turn, marker approach (after passing
        # the boxes), otherwise avoid/reverse/straight. (x gates limit the approach/U-turn zone.)
        if seen_marker and yaw_abs > 30:
            phase = "U-TURN"
        elif seen_marker and yaw_abs >= 150:
            phase = "drive"
        elif aruco_fresh and p[0] > 2.7 and not rev:
            phase = "ArUco approach"
        elif rev:
            phase = "REVERSE"
        elif abs(ang) >= 10:
            phase = "AVOID"
        else:
            phase = "drive"
        col = COL[phase]
        txt1 = "%s  speed=%+d  steer=%+d deg" % (phase, spd, ang)
        txt2 = "front sonar=%s cm  vision:%s  aruco:%s" % (
            S["front"] if S["front"] is not None else "?",
            "obstacle" if vis else "clear",
            "seen" if aruco_fresh else "-")
        cv2.rectangle(fr, (0, 0), (W, 44), (0, 0, 0), -1)
        cv2.putText(fr, txt1, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        cv2.putText(fr, txt2, (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1)
        fr = cv2.resize(fr, (420, 315))
        fpv_frames.append(cv2.cvtColor(fr, cv2.COLOR_BGR2RGB))
        samples.append((t, p[0], p[1], p[2], ang, phase))
    rate.sleep()

cmd_pub.publish(String(data="STOP"))
print("captured %d frames" % len(fpv_frames))

# ── FPV GIF ──
if fpv_frames:
    imgs = [PILImage.fromarray(f) for f in fpv_frames]
    imgs[0].save(OUT+"/demo_drive_fpv.gif", save_all=True, append_images=imgs[1:],
                 duration=170, loop=0, optimize=True)
    print("wrote demo_drive_fpv.gif")

# ── top-view trajectory GIF ──
BOXES  = [(1.4, -0.25, "#cc3030"), (2.1, 0.25, "#2ca02c")]   # 2 boxes (0.3m) in _demo_two_box.world
MARKER = (4.0, 0.0)                                          # front ArUco
PCOL = {"drive": "#1f77b4", "AVOID": "#ff7f0e", "REVERSE": "#d62728",
        "ArUco approach": "#17becf", "U-TURN": "#9467bd"}
xs = [s[1] for s in samples]; ys = [s[2] for s in samples]
top_frames = []
for i in range(len(samples)):
    t, x, y, yaw, ang, phase = samples[i]
    fig, axp = plt.subplots(figsize=(7.4, 2.8), dpi=100)
    axp.set_xlim(-1.0, 5.2); axp.set_ylim(-1.6, 1.6); axp.set_aspect("equal")
    axp.set_title("RODONG sim — 2-obstacle avoid -> ArUco -> U-turn", fontsize=9)
    axp.set_xlabel("x [m]", fontsize=8)
    for bx, by, bc in BOXES:
        axp.add_patch(Rectangle((bx-0.15, by-0.15), 0.3, 0.3, color=bc))
    axp.plot([MARKER[0]], [MARKER[1]], "ks", ms=8)
    axp.text(MARKER[0]+0.08, MARKER[1]+0.12, "ArUco", fontsize=7)
    axp.plot(xs[:i+1], ys[:i+1], "-", color="#1f77b4", lw=1.4, alpha=0.7)
    dx, dy = 0.40*math.cos(math.radians(yaw)), 0.40*math.sin(math.radians(yaw))
    c = PCOL.get(phase, "#1f77b4")
    axp.add_patch(FancyArrow(x, y, dx, dy, width=0.07, head_width=0.18,
                             head_length=0.14, color=c, length_includes_head=True))
    axp.text(-0.9, 1.42, "t=%.1fs  %s (steer %+d)" % (t, phase, ang), fontsize=8, color=c)
    fig.tight_layout()
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    top_frames.append(buf.copy())
    plt.close(fig)

if top_frames:
    imgs2 = [PILImage.fromarray(f) for f in top_frames]
    imgs2[0].save(OUT+"/demo_drive_top.gif", save_all=True, append_images=imgs2[1:],
                  duration=170, loop=0, optimize=True)
    print("wrote demo_drive_top.gif")
print("DONE")
