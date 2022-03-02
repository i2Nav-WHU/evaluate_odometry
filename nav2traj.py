# -*- coding: utf-8 -*-

"""
@File        : nav2traj.py
@Author      : hailiang
@Contact     : thl@whu.edu.cn
@Description : convert latitude-longitude-altitude and euler angle to trajectory file
@Version     : v1.0
"""

import numpy as np
import math as m

R2D = 180 / np.pi
D2R = np.pi / 180

RA = 6378137
E2E = 0.00669437999013
WIE = 7.2921151467e-5


def upmn(lat):
    sin2 = m.sin(lat) ** 2
    return [
        RA * (1 - E2E) / np.power(1 - E2E * sin2, 1.5),
        RA / np.sqrt(1 - E2E * sin2)
    ]


def euler2quat(euler):
    phi, theta, psi = euler
    sphi = m.sin(0.5 * phi)
    stheta = m.sin(0.5 * theta)
    spsi = m.sin(0.5 * psi)

    cphi = m.cos(0.5 * phi)
    ctheta = m.cos(0.5 * theta)
    cpsi = m.cos(0.5 * psi)

    a = cphi * ctheta * cpsi + sphi * stheta * spsi
    b = sphi * ctheta * cpsi - cphi * stheta * spsi
    c = cphi * stheta * cpsi + sphi * ctheta * spsi
    d = cphi * ctheta * spsi - sphi * stheta * cpsi
    q = np.array([a, b, c, d])
    return q / np.linalg.norm(q)


if __name__ == '__main__':
    navfile = '/home/hailiang/workspace/truth.nav'
    trajfile = '%s_traj.csv' % navfile.split('.')[0]

    nav = np.loadtxt(navfile)

    nav[:, 2:4] *= D2R
    nav[:, 8:11] *= D2R

    ref_pos = nav[0, 2:5]

    traj = np.zeros((len(nav), 8))
    traj[:, 0] = nav[:, 1]

    radm, radn = upmn(ref_pos[0])
    DR = np.array(
        [radm + ref_pos[2], (radn + ref_pos[2]) * m.cos(ref_pos[0]), -1])

    for k in range(len(traj)):
        traj[k, 1:4] = (nav[k, 2:5] - ref_pos) * DR
        q = euler2quat(nav[k, 8:11])
        traj[k, 4:7] = q[1:4]
        traj[k, 7] = q[0]

    np.savetxt(trajfile, traj, fmt='%0.4lf %0.4lf %0.4lf %0.4lf %0.6lf %0.6lf %0.6lf %0.6lf')
