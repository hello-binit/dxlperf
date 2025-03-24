#!/usr/bin/env python3

import time
import numpy as np
import stretch_body_ii.end_of_arm.end_of_arm_tools 
import math


r = stretch_body_ii.end_of_arm.end_of_arm_tools.EOA_Wrist_DW4_Tool_SG4()
assert r.startup(threaded=False)

#[r.get_joint(j).enable_velocity_ctrl() for j in ['wrist_yaw', 'wrist_pitch', 'wrist_roll', 'stretch_gripper']]
min_rate=10000
throwaway_cnt = 5
actual_dt_history = []
for _ in range(1000+throwaway_cnt):
    xx=abs(math.sin(time.time()))*1000.0
    x={'wrist_yaw':xx,
           'wrist_pitch':xx,
           'wrist_roll':xx,
           'stretch_gripper':xx}
    
    t0 = time.time()
    r.pull_status()
    r.stream_pos(x)
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

# dynamixel-sdk 3.7.31 on 3004 NUC @ Feb 19, 11am | Avg freq: 43.47
# dynamixel-sdk 3.7.31 on 3004 NUC @ Feb 19, 11am | Avg freq: 43.48
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 19, 12pm | Avg freq: 43.50
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 19, 12pm | Avg freq: 43.51
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 24,  8am | Avg freq: 43.53
# dynamixel-sdk 3.8.1  on 3004 NUC @ Mar  5,  3pm | Avg freq: 43.46
# dynamixel-sdk 3.8.1  on 3004 NUC @ Mar  5,  3pm | Avg freq: 43.49 | Avg CPU: 90% on 1 CPU

# dynamixel-sdk 3.8.1 w/usb_latency_timer=0ms on 3004 NUC @ Feb 24, 10:30am | Avg freq: 46.96 | Avg CPU: 50% across 2 CPUs
# dynamixel-sdk 3.8.1 w/usb_latency_timer=0ms on 3004 NUC @ Feb 24, 10:30am | Avg freq: 46.98
