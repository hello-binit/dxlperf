import time
import numpy as np
import stretch_body.dynamixel_hello_XL430

import dynamixel_sdk.port_handler as prh
prh.LATENCY_TIMER = 1

yaw = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430('wrist_yaw')
pitch = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430('wrist_pitch')
roll = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430('wrist_roll')
gripper = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430('stretch_gripper')
assert all([j.startup(threaded=False) for j in [yaw, pitch, roll, gripper]])
[j.disable_torque() for j in [yaw, pitch, roll, gripper]]

throwaway_cnt = 5
actual_dt_history = []
for _ in range(1000+throwaway_cnt):
    t0 = time.time()
    [j.pull_status() for j in [yaw, pitch, roll, gripper]]
    t1 = time.time()
    if throwaway_cnt > 0:
        throwaway_cnt -= 1
        continue
    curr_rate = 1.0 / (t1 - t0)
    print(curr_rate, [j.status['pos'] for j in [yaw, pitch, roll, gripper]])
    actual_dt_history.append([t0, t1])

actual_dt_history = np.array(actual_dt_history)
total_time = actual_dt_history[:, 1] - actual_dt_history[:, 0]
dt_mean = np.mean(total_time)
freq_mean = 1.0 / dt_mean
print()
print(f'Avg freq: {freq_mean:.2f}')

[j.stop() for j in [yaw, pitch, roll, gripper]]

# dynamixel-sdk 3.7.31 on 3004 NUC @ Feb 19, 11:30am | Avg freq: 25.17
# dynamixel-sdk 3.7.31 on 3004 NUC @ Feb 19, 11:30am | Avg freq: 25.57
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 19, 12pm    | Avg freq: 25.08
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 19, 12pm    | Avg freq: 25.52
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 19, 9pm     | Avg freq: 26.44
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 19, 10pm    | Avg freq: 26.74
# dynamixel-sdk 3.8.1  on 3004 NUC @ Feb 24, 8am     | Avg freq: 26.98
# dynamixel-sdk 3.8.1  on 3004 NUC @ Mar  5, 3pm     | Avg freq: 24.91 | Avg CPU: 90% on 1 CPU
# dynamixel-sdk 3.8.1  on 3004 NUC @ Mar  5, 3pm     | Avg freq: 26.01 | Avg CPU: 90% on 1 CPU

# dynamixel-sdk 3.8.1 w/usb_latency_timer=0ms on 3004 NUC @ Feb 24, 10:30am | Avg freq: 30.52 | Avg CPU: 60% across 2 CPUs
