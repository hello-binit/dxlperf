#!/usr/bin/env python3

import time
import numpy as np
import stretch_body_ii.end_of_arm.end_of_arm_tools 

# import dynamixel_sdk.port_handler as prh
# prh.LATENCY_TIMER = 64

# Profile CPU usage
import psutil
process = psutil.Process()
start_proc_times = process.cpu_times()
start_wall_time = time.time()

r = stretch_body_ii.end_of_arm.end_of_arm_tools.EOA_Wrist_DW4_Tool_SG4()
assert r.startup(threaded=False)

[r.get_joint(j).disable_torque() for j in ['wrist_yaw', 'wrist_pitch', 'wrist_roll', 'stretch_gripper']]
min_rate=1000
throwaway_cnt = 5
actual_dt_history = []
for _ in range(1000+throwaway_cnt):
    t0 = time.time()
    r.pull_status()
    t1 = time.time()
    if throwaway_cnt > 0:
        throwaway_cnt -= 1
        continue
    curr_rate = 1.0 / (t1 - t0)
    min_rate=min(min_rate,curr_rate)
    print(curr_rate, [r.status[j]['pos'] for j in ['wrist_yaw', 'wrist_pitch', 'wrist_roll', 'stretch_gripper']])
    actual_dt_history.append([t0, t1])

actual_dt_history = np.array(actual_dt_history)
total_time = actual_dt_history[:, 1] - actual_dt_history[:, 0]
dt_mean = np.mean(total_time)
freq_mean = 1.0 / dt_mean
print()
print(f'Avg freq: {freq_mean:.2f}')
print()
print(f'Min freq: {min_rate:.2f}')
r.stop()

# Calculate CPU usage
end_proc_times = process.cpu_times()
end_wall_time = time.time()

# User + System CPU time used by this process over entire run
used_cpu_time = (
    (end_proc_times.user - start_proc_times.user) +
    (end_proc_times.system - start_proc_times.system)
)

# Total elapsed (wall-clock) time
elapsed_time = end_wall_time - start_wall_time

# if fully occupies one core on a 4-core machine, it would read ~100%, but if it occupies all 4 cores, it would read ~400%.
cpu_usage_percent = used_cpu_time / elapsed_time * 100
print(f'Avg CPU: {cpu_usage_percent:.1f}%')

# dynamixel-sdk 3.7.31 on 3004 NUC @ Feb 19, 11am | Avg freq: 43.47
# dynamixel-sdk 3.7.31 on 3004 NUC @ Feb 19, 11am | Avg freq: 43.48
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 19, 12pm | Avg freq: 43.50
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 19, 12pm | Avg freq: 43.51
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 24,  8am | Avg freq: 43.53
# dynamixel-sdk 3.8.1  on 3004 NUC @ Mar  5,  3pm | Avg freq: 43.46
# dynamixel-sdk 3.8.1  on 3004 NUC @ Mar  5,  3pm | Avg freq: 43.49 | Avg CPU: 90% on 1 CPU

# dynamixel-sdk 3.8.1 w/usb_latency_timer=0ms on 3004 NUC @ Feb 24, 10:30am | Avg freq: 46.96 | Avg CPU: 50% across 2 CPUs
# dynamixel-sdk 3.8.1 w/usb_latency_timer=0ms on 3004 NUC @ Feb 24, 10:30am | Avg freq: 46.98
