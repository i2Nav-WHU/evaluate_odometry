# -*- coding: utf-8 -*-

"""
@File        : evaluate.py
@Author      : hailiang
@Contact     : thl@whu.edu.cn
@Description : evaluate the odometry
@Version     : v1.0
"""

import numpy as np
import matplotlib.pyplot as plt
from evo.core import metrics, sync, trajectory
from evo.tools import log, plot
import gpstime
import pprint
import copy
import quaternion

# 最大时间同步误差, 参考轨迹是200Hz, 间隔5ms
# Maximum time synchronization error, for 200Hz is 5ms
MAX_TIME_SYNC_DIFF = 0.005

# 相对误差统计间隔 (m)
# Relative error delta in meter
# RPE_DELTA = [10, 20, 50]
RPE_DELTA = [50, 100, 150, 200, 250, 300]

# 使用所有匹配对进行相对位姿统计
# use all pose pairs for RPE
IS_RPE_ALL_PAIRS = True

# 是否显示轨迹和误差曲线
# show trajectory and error profiles
IS_SHOW_PLOTS = True

# 仅显示APE
# show APE only
IS_SHOW_APE_ONLY = False

# 是否仅输出精简的RMSE统计结果
# print the statistic
IS_ONLY_RMSE = True

# 是否输出误差文件
# output the RMSE result to file
IS_SAVE_ERROR_FILE = True


def load_odo_file(odofile):
    traj = np.loadtxt(odofile)

    for k in range(len(traj)):
        week, sow = gpstime.unix_second_to_gps_time(traj[k, 0])
        traj[k, 0] = sow

    return traj


def time_alignment(traj, starttime, endtime):
    aligned = traj.copy()

    for k in range(len(aligned)):
        if aligned[0, 0] > starttime:
            break
        else:
            aligned = np.delete(aligned, 0, axis=0)

    for k in range(len(aligned)):
        if aligned[-1, 0] > endtime:
            aligned = np.delete(aligned, -1, axis=0)
        else:
            break

    return aligned


def pose_interpolation(pose0, pose1, midtime):
    if abs(pose0[0] - midtime) < 0.0001:
        pose0[0] = midtime
        return pose0
    elif abs(pose1[0] - midtime) < 0.0001:
        pose1[0] = midtime
        return pose1
    else:
        pose = np.zeros(8)
        pose[0] = midtime

        scale = (midtime - pose0[0]) / (pose1[0] - pose0[0])
        pose[1:4] = pose0[1:4] + (pose1[1:4] - pose0[1:4]) * scale

        q0 = np.array([pose0[7], pose0[4], pose0[5], pose0[6]])
        q1 = np.array([pose1[7], pose1[4], pose1[5], pose1[6]])

        quat0 = quaternion.from_float_array(q0)
        quat1 = quaternion.from_float_array(q1)

        q_1_0 = quat1.inverse() * quat0
        # 保证正方向旋转, 即小角度
        # make sure small angle
        if q_1_0.w < 0:
            q_1_0 *= -1

        vec = quaternion.as_rotation_vector(q_1_0)
        q_2_0 = quaternion.from_rotation_vector(vec * scale)
        quat2 = quat0 * q_2_0.inverse()

        pose[4:8] = np.array([quat2.x, quat2.y, quat2.z, quat2.w])

        return pose


def evaluate_ape(ref, est, outdir):
    # 平移误差
    # translation error
    pose_relation = metrics.PoseRelation.translation_part

    ape_metric = metrics.APE(pose_relation)
    ape_metric.process_data((ref, est))

    ape_stats = ape_metric.get_all_statistics()
    if not IS_ONLY_RMSE:
        print('\nAbsolute translation error:')
        pprint.pprint(ape_stats)

    translation_rmse = ape_stats['rmse']

    if IS_SHOW_PLOTS:
        fig = plt.figure()
        plot.error_array(fig.gca(), ape_metric.error, x_array=est.timestamps,
                         statistics={s: v for s, v in ape_stats.items() if s != "sse"},
                         name="APE", title="APE w.r.t. " + ape_metric.pose_relation.value, xlabel="$t$ (s)")
        plt.grid()

    ape_metric.get_result()

    error = None
    if IS_SAVE_ERROR_FILE:
        error = np.zeros((len(ape_metric.error), 3))
        error[:, 0] = est.timestamps
        error[:, 1] = ape_metric.error

    # 角度误差
    # rotation error
    pose_relation = metrics.PoseRelation.rotation_angle_deg

    ape_metric = metrics.APE(pose_relation)
    ape_metric.process_data((ref, est))

    ape_stats = ape_metric.get_all_statistics()
    if not IS_ONLY_RMSE:
        print('\nAbsolute rotation error:')
        pprint.pprint(ape_stats)

    rotation_rmse = ape_stats['rmse']

    if IS_SHOW_PLOTS:
        fig = plt.figure()
        plot.error_array(fig.gca(), ape_metric.error, x_array=est.timestamps,
                         statistics={s: v for s, v in ape_stats.items() if s != "sse"},
                         name="APE", title="APE w.r.t. " + ape_metric.pose_relation.value, xlabel="$t$ (s)")
        plt.grid()

    if IS_SAVE_ERROR_FILE:
        error[:, 2] = ape_metric.error
        np.savetxt(outdir + '/ape_error.txt', error, fmt='%-15.4lf %-9.4lf % -9.4lf')

    return translation_rmse, rotation_rmse


def evaluate_rpe(ref, est, rpe_delta, outdir):
    # 相对误差统计度量
    # RPE delta and unit
    delta = rpe_delta
    delta_unit = metrics.Unit.meters

    rel_delta_tol = 0.002

    pose_relation = metrics.PoseRelation.translation_part

    rpe_metric = metrics.RPE(pose_relation, delta, delta_unit, rel_delta_tol, IS_RPE_ALL_PAIRS)
    rpe_metric.process_data((ref, est))

    rpe_stats = rpe_metric.get_all_statistics()
    if not IS_ONLY_RMSE:
        print('\nRelative translastion error (%0.0lfm):' % rpe_delta)
        pprint.pprint(rpe_stats)

    translation_rmse = rpe_stats['rmse']

    if IS_SHOW_PLOTS and not IS_SHOW_APE_ONLY:
        traj_est_plot = copy.deepcopy(est)
        traj_est_plot.reduce_to_ids(rpe_metric.delta_ids)

        fig = plt.figure()
        plot.error_array(fig.gca(), rpe_metric.error, x_array=traj_est_plot.timestamps,
                         statistics={s: v for s, v in rpe_stats.items() if s != "sse"},
                         name="RPE",
                         title="RPE w.r.t. " + rpe_metric.pose_relation.value + (" (delta=%0.0lfm)" % rpe_delta),
                         xlabel="$t$ (s)")
        plt.grid()

    error = None
    if IS_SAVE_ERROR_FILE:
        traj_est_err = copy.deepcopy(est)
        traj_est_err.reduce_to_ids(rpe_metric.delta_ids)

        error = np.zeros((len(rpe_metric.error), 3))
        error[:, 0] = traj_est_err.timestamps
        error[:, 1] = rpe_metric.error

    pose_relation = metrics.PoseRelation.rotation_angle_deg

    rpe_metric = metrics.RPE(pose_relation, delta, delta_unit, rel_delta_tol, IS_RPE_ALL_PAIRS)
    rpe_metric.process_data((ref, est))

    rpe_stats = rpe_metric.get_all_statistics()
    if not IS_ONLY_RMSE:
        print('\nRelative rotation error (%0.0lfm):' % rpe_delta)
        pprint.pprint(rpe_stats)

    rotation_rmse = rpe_stats['rmse']

    if IS_SHOW_PLOTS and not IS_SHOW_APE_ONLY:
        traj_est_plot = copy.deepcopy(est)
        traj_est_plot.reduce_to_ids(rpe_metric.delta_ids)

        fig = plt.figure()
        plot.error_array(fig.gca(), rpe_metric.error, x_array=traj_est_plot.timestamps,
                         statistics={s: v for s, v in rpe_stats.items() if s != "sse"},
                         name="RPE",
                         title="RPE w.r.t. " + rpe_metric.pose_relation.value + (" (delta=%0.0lfm)" % rpe_delta),
                         xlabel="$t$ (s)")
        plt.grid()

    if IS_SAVE_ERROR_FILE:
        error[:, 2] = rpe_metric.error
        np.savetxt(outdir + '/rpe_error_%0.0lfm.txt' % rpe_delta, error, fmt='%-15.4lf %-9.4lf %-9.4lf')

    return translation_rmse, rotation_rmse


def show_aligned_traj(ref, est):
    fig = plt.figure()
    traj_by_label = {
        "estimate": est,
        "reference": ref
    }
    plot.trajectories(fig, traj_by_label, plot.PlotMode.yx)
    plt.grid()


def numpy_to_trajectory(traj):
    stamps = traj[:, 0]
    xyz = traj[:, 1:4]
    quat = traj[:, 4:]  # xyzw
    quat = np.roll(quat, 1, axis=1)  # wxyz

    return trajectory.PoseTrajectory3D(xyz, quat, stamps)


def trajectory_align(ref, est):
    # convert to PoseTrajectory3D object
    traj_ref_raw = numpy_to_trajectory(ref)
    traj_est_raw = numpy_to_trajectory(est)

    # 时间对齐
    # time alignment
    traj_ref, traj_est = sync.associate_trajectories(traj_ref_raw, traj_est_raw, MAX_TIME_SYNC_DIFF)

    # 轨迹对齐, SE3对齐
    # SE3 alignment
    traj_est_aligned = copy.deepcopy(traj_est)
    traj_est_aligned.align(traj_ref, correct_scale=False, correct_only_scale=False)

    return traj_ref, traj_est_aligned


def evaluate(ref, est, outdir):
    # 设置evo日志输出
    # evo logging configuration
    log.configure_logging(verbose=False, debug=False, silent=False)

    # 轨迹时间和位置对齐
    # time and tranjectory alignment
    traj_ref, traj_est = trajectory_align(ref, est)

    # 显示轨迹
    # show trajectory
    if IS_SHOW_PLOTS:
        show_aligned_traj(traj_ref, traj_est)

    # APE
    ape_trans_rmse, ape_rota_rmse = evaluate_ape(traj_ref, traj_est, outdir)

    print('Absolute RMSE (deg / m): %0.3lf / %0.3lf' % (ape_rota_rmse, ape_trans_rmse))

    # RPE
    for rpe_delta in RPE_DELTA:
        rpe_trans_rmse, rpe_rota_rmse = evaluate_rpe(traj_ref, traj_est, rpe_delta, outdir)
        print('Relative RMSE in %0.0lfm (deg / %%): %0.2lf / %0.2lf' % (
            rpe_delta, rpe_rota_rmse, rpe_trans_rmse / rpe_delta * 100))

    if IS_SHOW_PLOTS:
        # dispaly the figures
        plt.show()
