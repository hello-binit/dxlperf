//
// *********     C++ SDK Fast Sync Reader FPS      *********
//
//
// This example is designed for a Stretch 3, which has a wrist chain consisting of:
// [Dynamixel ID:013] ping Succeeded. Dynamixel model : XC430-W240. Baud 115200
// [Dynamixel ID:014] ping Succeeded. Dynamixel model : XL430-W250. Baud 115200
// [Dynamixel ID:015] ping Succeeded. Dynamixel model : XM540-W270. Baud 115200
// [Dynamixel ID:016] ping Succeeded. Dynamixel model : XM430-W350. Baud 115200
//

#include <iostream>
#include <chrono>
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
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");
    } else {
        printf("Failed to open the port!\n");
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    } else {
        printf("Failed to change the baudrate!\n");
        return 0;
    }

    // Disable torque
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
    }

    // Initialize GroupFastSyncReads
    dynamixel::GroupFastSyncRead groupread_pos(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    dynamixel::GroupFastSyncRead groupread_vel(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
    dynamixel::GroupFastSyncRead groupread_load(portHandler, packetHandler, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
    dynamixel::GroupFastSyncRead groupread_temp(portHandler, packetHandler, ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE);
    dynamixel::GroupFastSyncRead groupread_errs(portHandler, packetHandler, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS);
    for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
        if (!groupread_pos.addParam(dxl_id)) {
            printf("[ID:%03d] position groupFastSyncRead addparam failed", dxl_id);
            return 0;
        }
        if (!groupread_vel.addParam(dxl_id)) {
            printf("[ID:%03d] velocity groupFastSyncRead addparam failed", dxl_id);
            return 0;
        }
        if (!groupread_load.addParam(dxl_id)) {
            printf("[ID:%03d] current groupFastSyncRead addparam failed", dxl_id);
            return 0;
        }
        if (!groupread_temp.addParam(dxl_id)) {
            printf("[ID:%03d] temperature groupFastSyncRead addparam failed", dxl_id);
            return 0;
        }
        if (!groupread_errs.addParam(dxl_id)) {
            printf("[ID:%03d] hardware errors groupFastSyncRead addparam failed", dxl_id);
            return 0;
        }
    }

    // 2. Pull status for each joint
    int throwaway_cnt = 5;
    double dxl_present_position[4] = {0.0, 0.0, 0.0, 0.0};
    double diff[1000];
    int status_mux_id = 0;
    int zeros[4] = {YAW_ZERO, GRIPPER_ZERO, PITCH_ZERO, ROLL_ZERO};
    double pols[4] = {YAW_POLARITY, GRIPPER_POLARITY, PITCH_POLARITY, ROLL_POLARITY};
    double grs[4] = {YAW_GR, GRIPPER_GR, PITCH_GR, ROLL_GR};
    for (int i = 0; i < 1000+throwaway_cnt; i++) {
        auto start = std::chrono::high_resolution_clock::now();

        // Read status
        dxl_comm_result = groupread_pos.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return 0;
        } else {
            for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
                // Check error
                if (groupread_pos.getError(dxl_id, &dxl_error)) {
                    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
                    return 0;
                }

                // Check is data available
                if (!groupread_pos.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
                    printf("[ID:%03d] position groupFastSyncRead getdata failed", dxl_id);
                    return 0;
                }

                // parse position
                int o = dxl_id - BASE_DXL_ID;
                int32_t pos_ticks = groupread_pos.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
                dxl_present_position[o] = ticks_to_world_rad(pos_ticks, zeros[o], pols[o], grs[o]);
            }
        }
        dxl_comm_result = groupread_vel.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return 0;
        } else {
            for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
                // Check error
                if (groupread_vel.getError(dxl_id, &dxl_error)) {
                    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
                    return 0;
                }

                // Check is data available
                if (!groupread_vel.isAvailable(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)) {
                    printf("[ID:%03d] velocity groupFastSyncRead getdata failed", dxl_id);
                    return 0;
                }

                // parse velocity
                int o = dxl_id - BASE_DXL_ID;
                int32_t vel_ticks = groupread_vel.getData(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
                ticks_to_world_rad_per_sec(vel_ticks, pols[o], grs[o]);
            }
        }
        if (status_mux_id == 0) {
            dxl_comm_result = groupread_load.txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS) {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                return 0;
            } else {
                for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
                    // Check error
                    if (groupread_load.getError(dxl_id, &dxl_error)) {
                        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
                        return 0;
                    }

                    // Check is data available
                    if (!groupread_load.isAvailable(dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)) {
                        printf("[ID:%03d] current groupFastSyncRead getdata failed", dxl_id);
                        return 0;
                    }

                    // parse load/current
                    // int o = dxl_id - BASE_DXL_ID;
                    int16_t load_ticks = groupread_load.getData(dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
                }
            }
        }
        if (status_mux_id == 1) {
            dxl_comm_result = groupread_temp.txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS) {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                return 0;
            } else {
                for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
                    // Check error
                    if (groupread_temp.getError(dxl_id, &dxl_error)) {
                        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
                        return 0;
                    }

                    // Check is data available
                    if (!groupread_temp.isAvailable(dxl_id, ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE)) {
                        printf("[ID:%03d] temperature groupFastSyncRead getdata failed", dxl_id);
                        return 0;
                    }

                    // parse temperature
                    // int o = dxl_id - BASE_DXL_ID;
                    int8_t temp = groupread_temp.getData(dxl_id, ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE);
                }
            }
        }
        if (status_mux_id == 2) {
            dxl_comm_result = groupread_errs.txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS) {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                return 0;
            } else {
                for (int dxl_id = BASE_DXL_ID; dxl_id < 17; dxl_id++) {
                    // Check error
                    if (groupread_errs.getError(dxl_id, &dxl_error)) {
                        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
                        return 0;
                    }

                    // Check is data available
                    if (!groupread_errs.isAvailable(dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS)) {
                        printf("[ID:%03d] hardware errors groupFastSyncRead getdata failed", dxl_id);
                        return 0;
                    }

                    // parse hardware errors
                    // int o = dxl_id - BASE_DXL_ID;
                    int8_t hw_errors = groupread_errs.getData(dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS);
                }
            }
        }
        status_mux_id = (status_mux_id + 1) % 3;

        auto end = std::chrono::high_resolution_clock::now();
        if (throwaway_cnt > 0) {
            throwaway_cnt -= 1;
            continue;
        }
        std::chrono::duration<double> elapsed = end - start;
        double elapsed_time = elapsed.count();
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
    portHandler->closePort();
    return 0;
}

// C++ libdxl 3.8.1 on 3004 NUC @ Mar 5, 9pm  | Avg freq: 59.28
// C++ libdxl 3.8.1 on 3004 NUC @ Mar 5, 9pm  | Avg freq: 59.27 | Avg CPU: 8% on 1 CPU
// C++ libdxl 3.8.1 on 3004 NUC @ Mar 5, 9pm  | Avg freq: 59.27

// C++ libdxl 3.8.1 w/usb_latency_timer=0ms on 3004 NUC @ Mar 5, 9pm  | Avg freq: 67.51 | Avg CPU: 27% on 1 CPU
// C++ libdxl 3.8.1 w/usb_latency_timer=0ms on 3004 NUC @ Mar 5, 9pm  | Avg freq: 67.56
