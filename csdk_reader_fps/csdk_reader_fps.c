//
// *********     C SDK Reader FPS      *********
//
//
// This example is designed for a Stretch 3, which has a wrist chain consisting of:
// [Dynamixel ID:013] ping Succeeded. Dynamixel model : XC430-W240. Baud 115200
// [Dynamixel ID:014] ping Succeeded. Dynamixel model : XL430-W250. Baud 115200
// [Dynamixel ID:015] ping Succeeded. Dynamixel model : XM540-W270. Baud 115200
// [Dynamixel ID:016] ping Succeeded. Dynamixel model : XM430-W350. Baud 115200
//

#include <stdio.h>
#include <time.h>
#include "dynamixel_sdk.h"

// Protocol version
#define PROTOCOL_VERSION                2.0

// Comm setting
#define BASE_DXL_ID                     13                  // Wrist chain Dynamixel IDs start at 13
#define BAUDRATE                        115200
#define DEVICENAME                      "/dev/ttyUSB2"

// Control table
#define ADDR_PRESENT_POSITION           132
#define ADDR_PRESENT_VELOCITY           128
#define ADDR_PRESENT_LOAD               126
#define ADDR_PRESENT_TEMPERATURE        146
#define ADDR_HARDWARE_ERROR_STATUS       70
#define ADDR_TORQUE_ENABLE               64

// Data Byte Length
#define LEN_GOAL_POSITION                 4
#define LEN_PRESENT_POSITION              4
#define LEN_PRESENT_VELOCITY              4
#define LEN_PRESENT_LOAD                  2
#define LEN_PRESENT_TEMPERATURE           1
#define LEN_HARDWARE_ERROR_STATUS         1

// Servo calibration (for Stretch 3004)
#define YAW_ZERO                       7005
#define PITCH_ZERO                     1024
#define ROLL_ZERO                      2048
#define GRIPPER_ZERO                   2522
#define YAW_POLARITY                   -1.0
#define PITCH_POLARITY                 -1.0
#define ROLL_POLARITY                   1.0
#define GRIPPER_POLARITY                1.0
#define YAW_GR                          2.4
#define PITCH_GR                        1.0
#define ROLL_GR                         1.0
#define GRIPPER_GR                      1.0


double deg_to_rad(double deg) {
    return deg * 3.14159265 / 180.0;
}

double ticks_to_rad(int32_t ticks) {
    return deg_to_rad(360.0 * ticks / 4096.0);
}

double ticks_to_world_rad(int32_t ticks, int32_t zero, double polarity, double gear_ratio) {
    return polarity * ticks_to_rad(ticks - zero) / gear_ratio;
}

double ticks_to_rad_per_sec(int32_t ticks) {
    return deg_to_rad((ticks * 0.229) * 360.0/60.0);
}

double ticks_to_world_rad_per_sec(int32_t ticks, double polarity, double gear_ratio) {
    return polarity * ticks_to_rad_per_sec(ticks) / gear_ratio;
}


int main() {
    // 1. Init four servos
    int port_num = portHandler(DEVICENAME);
    packetHandler();                                // Initialize PacketHandler Structs

    // Open port
    if (openPort(port_num)) {
        printf("Succeeded to open the port!\n");
    } else {
        printf("Failed to open the port!\n");
        return 0;
    }

    // Set port baudrate
    if (setBaudRate(port_num, BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    } else {
        printf("Failed to change the baudrate!\n");
        return 0;
    }

    // Disable torque
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error;                              // Dynamixel error
    for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
        write1ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_TORQUE_ENABLE, 0);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
            printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        } else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0) {
            printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
        }
    }

    // Initialize groupSyncRead structs
    int groupread_pos  = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    int groupread_vel  = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
    int groupread_load = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
    int groupread_temp = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE);
    int groupread_errs = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS);
    for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
        if (!groupSyncReadAddParam(groupread_pos, dxl_id)) {
            printf("[ID:%03d] position groupSyncRead addparam failed", dxl_id);
            return 0;
        }
        if (!groupSyncReadAddParam(groupread_vel, dxl_id)) {
            printf("[ID:%03d] velocity groupSyncRead addparam failed", dxl_id);
            return 0;
        }
        if (!groupSyncReadAddParam(groupread_load, dxl_id)) {
            printf("[ID:%03d] current groupSyncRead addparam failed", dxl_id);
            return 0;
        }
        if (!groupSyncReadAddParam(groupread_temp, dxl_id)) {
            printf("[ID:%03d] temperature groupSyncRead addparam failed", dxl_id);
            return 0;
        }
        if (!groupSyncReadAddParam(groupread_errs, dxl_id)) {
            printf("[ID:%03d] hardware errors groupSyncRead addparam failed", dxl_id);
            return 0;
        }
    }

    // 2. Pull status for each joint
    struct timespec start, end;
    int throwaway_cnt = 5;
    double dxl_present_position[4] = {0.0, 0.0, 0.0, 0.0};
    double diff[1000];
    int status_mux_id = 0;
    int zeros[4] = {YAW_ZERO, GRIPPER_ZERO, PITCH_ZERO, ROLL_ZERO};
    double pols[4] = {YAW_POLARITY, GRIPPER_POLARITY, PITCH_POLARITY, ROLL_POLARITY};
    double grs[4] = {YAW_GR, GRIPPER_GR, PITCH_GR, ROLL_GR};
    for (int i = 0; i < 1000+throwaway_cnt; i++) {
        clock_gettime(CLOCK_MONOTONIC, &start);

        // Read status
        groupSyncReadTxRxPacket(groupread_pos);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
            printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        } else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0) {
            printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
        } else {
            for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
                if (!groupSyncReadIsAvailable(groupread_pos, dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
                    printf("[ID:%03d] position groupSyncRead getdata failed", dxl_id);
                    return 0;
                }

                // parse position
                int o = dxl_id - BASE_DXL_ID;
                int32_t pos_ticks = groupSyncReadGetData(groupread_pos, dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
                dxl_present_position[o] = ticks_to_world_rad(pos_ticks, zeros[o], pols[o], grs[o]);
            }
        }
        groupSyncReadTxRxPacket(groupread_vel);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
            printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        } else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0) {
            printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
        } else {
            for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
                if (!groupSyncReadIsAvailable(groupread_vel, dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)) {
                    printf("[ID:%03d] velocity groupSyncRead getdata failed", dxl_id);
                    return 0;
                }

                // parse velocity
                int o = dxl_id - BASE_DXL_ID;
                int32_t vel_ticks = groupSyncReadGetData(groupread_vel, dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
                ticks_to_world_rad_per_sec(vel_ticks, pols[o], grs[o]);
            }
        }
        if (status_mux_id == 0) {
            groupSyncReadTxRxPacket(groupread_load);
            if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
                printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            } else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0) {
                printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
            } else {
                for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
                    if (!groupSyncReadIsAvailable(groupread_load, dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)) {
                        printf("[ID:%03d] current groupSyncRead getdata failed", dxl_id);
                        return 0;
                    }

                    // parse load/current
                    // int o = dxl_id - BASE_DXL_ID;
                    int16_t load_ticks = groupSyncReadGetData(groupread_load, dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
                }
            }
        }
        if (status_mux_id == 1) {
            groupSyncReadTxRxPacket(groupread_temp);
            if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
                printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            } else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0) {
                printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
            } else {
                for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
                    if (!groupSyncReadIsAvailable(groupread_temp, dxl_id, ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE)) {
                        printf("[ID:%03d] temperature groupSyncRead getdata failed", dxl_id);
                        return 0;
                    }

                    // parse temperature
                    // int o = dxl_id - BASE_DXL_ID;
                    int8_t temp = groupSyncReadGetData(groupread_temp, dxl_id, ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE);
                }
            }
        }
        if (status_mux_id == 2) {
            groupSyncReadTxRxPacket(groupread_errs);
            if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
                printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            } else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0) {
                printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
            } else {
                for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
                    if (!groupSyncReadIsAvailable(groupread_errs, dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS)) {
                        printf("[ID:%03d] hardware errors groupSyncRead getdata failed", dxl_id);
                        return 0;
                    }

                    // parse hardware errors
                    // int o = dxl_id - BASE_DXL_ID;
                    int8_t hw_errors = groupSyncReadGetData(groupread_errs, dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS);
                }
            }
        }
        status_mux_id = (status_mux_id + 1) % 3;

        clock_gettime(CLOCK_MONOTONIC, &end);
        if (throwaway_cnt > 0) {
            throwaway_cnt -= 1;
            continue;
        }
        double elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1E9;
        printf("%.2f [%.2f, %.2f, %.2f, %.2f]\n", 1 / elapsed_time, dxl_present_position[0], dxl_present_position[2], dxl_present_position[3], dxl_present_position[1]);
        diff[i] = elapsed_time;
    }

    // 3. Calculate rate
    double acc = 0.0;
    for (int i = 0; i < 1000; i++) {
        acc += diff[i];
    }
    double dt = acc / 1000;
    double fps = 1.0 / dt;
    printf("\nAvg freq: %.2f\n", fps);

    // 4. Shutdown
    closePort(port_num);
    return 0;
}


// C libdxl 3.8.1 on 3004 NUC @ Mar 5, 5pm  | Avg freq: 44.00
// C libdxl 3.8.1 on 3004 NUC @ Mar 5, 5pm  | Avg freq: 43.90 | Avg CPU: 10% on 1 CPU
// C libdxl 3.8.1 on 3004 NUC @ Mar 5, 5pm  | Avg freq: 44.01 | Avg CPU: 5% on 1 CPU
