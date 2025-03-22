import time
import psutil

# Profile CPU usage
process = psutil.Process()
start_proc_times = process.cpu_times()
start_wall_time = time.time()

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

# dynamixel-sdk 3.7.31 on 3004 NUC12 @ Feb 19, 11:30am | Avg freq: 25.17
# dynamixel-sdk 3.7.31 on 3004 NUC12 @ Feb 19, 11:30am | Avg freq: 25.57
# dynamixel-sdk 3.8.1  on 3004 NUC12 @ Feb 19, 12pm    | Avg freq: 25.08
# dynamixel-sdk 3.8.1  on 3004 NUC12 @ Feb 19, 12pm    | Avg freq: 25.52
# dynamixel-sdk 3.8.1  on 3004 NUC12 @ Feb 19, 9pm     | Avg freq: 26.44
# dynamixel-sdk 3.8.1  on 3004 NUC12 @ Feb 19, 10pm    | Avg freq: 26.74
# dynamixel-sdk 3.8.1  on 3004 NUC12 @ Feb 24, 8am     | Avg freq: 26.98
# dynamixel-sdk 3.8.1  on 3004 NUC12 @ Mar  5, 3pm     | Avg freq: 24.91 | Avg CPU: 90% on 1 CPU
# dynamixel-sdk 3.8.1  on 3004 NUC12 @ Mar  5, 3pm     | Avg freq: 26.01 | Avg CPU: 90% on 1 CPU

# dynamixel-sdk 3.8.1 w/usb_latency_timer=0ms on 3004 NUC12 @ Feb 24, 10:30am | Avg freq: 30.52 | Avg CPU: 60% across 2 CPUs


# dynamixel-sdk 3.7.31 on 2046 NUC12 @ Mar 18, 4pm | Avg freq: 26.35 | Avg CPU: 58% on 1 CPU
# dynamixel-sdk 3.7.31 on 2046 NUC12 @ Mar 18, 4pm | Avg freq: 26.20 | Avg CPU: 20% on 3 CPUs
# dynamixel-sdk 3.7.31 on 2046 NUC12 @ Mar 18, 5pm | Avg freq: 26.54 | Avg CPU: 96.1%
# dynamixel-sdk 3.7.31 on 2046 NUC12 @ Mar 18, 5pm | Avg freq: 26.27 | Avg CPU: 95.5%
