# -*- coding: utf-8 -*-

"""
@File        : evaluate_absolute.py
@Author      : hailiang
@Contact     : thl@whu.edu.cn
@Description : evaluate the absolute trajectory
@Version     : v1.0
"""

import matplotlib.pyplot as plt
import numpy as np
import evaluate

truth_file = 'path/truth_traj.csv'
odo_file = 'path/trajectory.csv'

truth_traj = np.loadtxt(truth_file)
# odo_traj = evaluate.load_odo_file(odo_file)
odo_traj = np.loadtxt(odo_file)

ts = 0
te = 0
if truth_traj[0, 0] > odo_traj[0, 0]:
    ts = truth_traj[0, 0]
else:
    ts = odo_traj[1, 0]

if truth_traj[-1, 0] > odo_traj[-1, 0]:
    te = odo_traj[-2, 0]
else:
    te = truth_traj[-1, 0]

# Time alignment
odo_traj_aligned = evaluate.time_alignment(odo_traj, ts, te)

truth_traj_aligned = np.zeros(odo_traj_aligned.shape)

index = 0
for k in range(len(truth_traj)):
    if truth_traj[k, 0] > odo_traj_aligned[index, 0]:
        truth_traj_aligned[index] = evaluate.pose_interpolation(truth_traj[k - 1], truth_traj[k],
                                                                odo_traj_aligned[index, 0])
        index += 1
    else:
        continue

    if index >= len(odo_traj_aligned):
        break

truth_traj_aligned = evaluate.numpy_to_trajectory(truth_traj_aligned)
odo_traj_aligned = evaluate.numpy_to_trajectory(odo_traj_aligned)

# Align to the original point
# odo_traj_aligned.align_origin(truth_traj_aligned)
# SE(3) alignment
odo_traj_aligned.align(truth_traj_aligned)

stamp = odo_traj_aligned.timestamps

# Position error
poserr = odo_traj_aligned.positions_xyz - truth_traj_aligned.positions_xyz

plt.figure('poserr')
plt.plot(stamp, poserr[:, 0])
plt.plot(stamp, poserr[:, 1])
plt.plot(stamp, poserr[:, 2])
plt.legend(['X', 'Y', 'Z'])
plt.xlabel('Time ($s$)')
plt.ylabel('Error ($m$)')
plt.grid()
plt.tight_layout()

# Attitude error
odo_euler = odo_traj_aligned.get_orientations_euler()
truth_euler = truth_traj_aligned.get_orientations_euler()

atterr = (odo_euler - truth_euler) * 180.0 / np.pi

for k in range(len(atterr)):
    if atterr[k, 2] > 180:
        atterr[k, 2] -= 360
    elif atterr[k, 2] < -180:
        atterr[k, 2] += 360

plt.figure('atterr')
plt.plot(stamp, atterr[:, 0])
plt.plot(stamp, atterr[:, 1])
plt.plot(stamp, atterr[:, 2])
plt.legend(['X', 'Y', 'Z'])
plt.xlabel('Time ($s$)')
plt.ylabel('Error ($deg$)')
plt.grid()
plt.tight_layout()

plt.show()
