#!/usr/bin/env python3
 
import time
import numpy as np
import stretch_body_ii.core.feetech_SM_hello 
import stretch_body_ii.core.feetech.port_handler as prh
import math

prh.LATENCY_TIMER = 10

yaw =  stretch_body_ii.core.feetech_SM_hello.FeetechSMHello('wrist_yaw')
pitch = stretch_body_ii.core.feetech_SM_hello.FeetechSMHello('wrist_pitch')
roll = stretch_body_ii.core.feetech_SM_hello.FeetechSMHello('wrist_roll')
gripper = stretch_body_ii.core.feetech_SM_hello.FeetechSMHello('stretch_gripper')

assert all([j.startup(threaded=False) for j in [yaw, pitch, roll, gripper]])
#[j.enable_velocity_ctrl() for j in [yaw, pitch, roll, gripper]]

throwaway_cnt = 5
actual_dt_history = []
min_rate=1000

for _ in range(1000+throwaway_cnt):
    t0 = time.time()
    [j.pull_status() for j in [yaw, pitch, roll, gripper]]
    v_des=math.sin(time.time())*3.0
    [j.move_to(v_des) for j in [yaw, pitch, roll, gripper]]
    t1 = time.time()
    if throwaway_cnt > 0:
        throwaway_cnt -= 1
        continue
    curr_rate = 1.0 / (t1 - t0)
    min_rate=min(min_rate,curr_rate)
    print(curr_rate, [j.status['pos'] for j in [yaw, pitch, roll, gripper]])
    actual_dt_history.append([t0, t1])

[j.disable_torque() for j in [yaw, pitch, roll, gripper]]

actual_dt_history = np.array(actual_dt_history)
total_time = actual_dt_history[:, 1] - actual_dt_history[:, 0]
dt_mean = np.mean(total_time)
freq_mean = 1.0 / dt_mean
print()
print(f'Avg freq: {freq_mean:.2f}')
print()
print(f'Min freq: {min_rate:.2f}')

[j.stop() for j in [yaw, pitch, roll, gripper]]

