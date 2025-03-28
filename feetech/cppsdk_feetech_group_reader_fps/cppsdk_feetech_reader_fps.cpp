#include <iostream>
#include <csignal>
#include <cmath>
#include "SCServo.h"
#include "time.h"
#include <chrono>
#include <sys/resource.h>  // for getrusage()
#include <sys/time.h>      // for struct rusage's timeval
SMS_STS sm_st;

u8 ID[4] = {20,21,22,23};
uint8_t rxPacket[100];

void signalHandler(int signum) {
    if (signum == SIGINT) {
		std::cout<<"Terminated!"<<std::endl;
        sm_st.end();
        exit(0);
    }
}


int16_t Position;
int16_t Speed;
int16_t Temp;

int main(int argc, char **argv)
{
        // 0. Profile CPU usage
        struct rusage usage_start;
        getrusage(RUSAGE_SELF, &usage_start);
        auto real_start = std::chrono::steady_clock::now();

    if(!sm_st.begin(1000000, "/dev/hello-feetech-wrist")){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }

    

    signal(SIGINT, signalHandler);
    
    struct timespec start, end;
    int throwaway_cnt = 5;
    double diff[1000];
    double vmin=10000;
    int16_t Position[4];
    int16_t Speed[4];
    int16_t Temp[4];

    //NOTE: segfaults after about 2K cycles
    //NOTE: Can group syncReadPacketTx for registers that are adjacent in memory to save a syncReadPacketTx
    for (int t = 0;t < 1000+throwaway_cnt; t++) {
	
        
        clock_gettime(CLOCK_MONOTONIC, &start);

        sm_st.syncReadBegin(sizeof(ID), 2);

        sm_st.syncReadPacketTx(ID, sizeof(ID), SMS_STS_PRESENT_POSITION_L, 2);
		for(uint8_t i=0; i<sizeof(ID); i++){
			if(!sm_st.syncReadPacketRx(ID[i], rxPacket)){
				std::cout<<"ID:"<<(int)ID[i]<<" sync read error1!"<<std::endl;
				continue;
			}
			Position[i] = sm_st.syncReadRxPacketToWrod(15);
		}

        sm_st.syncReadPacketTx(ID, sizeof(ID), SMS_STS_PRESENT_SPEED_L, 2);
		for(uint8_t i=0; i<sizeof(ID); i++){
			if(!sm_st.syncReadPacketRx(ID[i], rxPacket)){
				std::cout<<"ID:"<<(int)ID[i]<<" sync read error2!"<<std::endl;
				continue;
			}
			Speed[i] = sm_st.syncReadRxPacketToWrod(15);
			
		}

        sm_st.syncReadEnd();
        sm_st.syncReadBegin(sizeof(ID), 1);

        sm_st.syncReadPacketTx(ID, sizeof(ID), SMS_STS_PRESENT_TEMPERATURE, 1);
		for(uint8_t i=0; i<sizeof(ID); i++){
			if(!sm_st.syncReadPacketRx(ID[i], rxPacket)){
				std::cout<<"ID:"<<(int)ID[i]<<" sync read error3!"<<std::endl;
				continue;
			}
			Temp[i] = sm_st.syncReadRxPacketToByte();
		}

        sm_st.syncReadEnd();

        /*sm_st.syncReadPacketTx(ID, sizeof(ID), SMS_STS_PRESENT_TEMPERATURE, 2);//同步读指令包发送
		for(uint8_t i=0; i<sizeof(ID); i++){
			//接收ID[i]同步读返回包
			if(!sm_st.syncReadPacketRx(ID[i], rxPacket)){
				std::cout<<"ID:"<<(int)ID[i]<<" sync read error!"<<std::endl;
				continue;//接收解码失败
			}
			Temp = sm_st.syncReadRxPacketToWrod(15);//解码两个字节 bit15为方向位,参数=0表示无方向位
			//std::cout<<"ID:"<<int(ID[i])<<" Temp:"<<Temp<<std::endl;
		}*/

        clock_gettime(CLOCK_MONOTONIC, &end);
        for(uint8_t i=0; i<sizeof(ID); i++){
        std::cout<<"ID:"<<int(ID[i])<<" Position:"<<Position[i]<<std::endl;
        std::cout<<"ID:"<<int(ID[i])<<" Speed:"<<Speed[i]<<std::endl;
        std::cout<<"ID:"<<int(ID[i])<<" Temp:"<<Temp[i]<<std::endl;}

        if (throwaway_cnt > 0) {
            throwaway_cnt -= 1;
            continue;
        }

        double elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1E9;
        printf("%.2f\n", 1 / elapsed_time);
        diff[t] = elapsed_time;
        vmin=fmin(vmin,1/elapsed_time);

        std::cout<<"--------------------------------"<<t<<"--------------------------------"<<std::endl;
        //sleep(1);
	}

    // 3. Calculate rate
    double acc = 0.0;
    for (int i = 0; i < 1000; i++) {
        acc += diff[i];
    }
    double dt = acc / 1000;
    double fps = 1.0 / dt;
    printf("\nAvg freq: %.2f\n", fps);
    printf("\Min freq: %.2f\n", vmin);
 

    // 5. Calculate CPU usage
    struct rusage usage_end;
    getrusage(RUSAGE_SELF, &usage_end);
    auto real_end = std::chrono::steady_clock::now();

    // CPU time (user + sys)
    double user_start = usage_start.ru_utime.tv_sec + (usage_start.ru_utime.tv_usec / 1e6);
    double user_end_t = usage_end.ru_utime.tv_sec   + (usage_end.ru_utime.tv_usec / 1e6);

    double sys_start  = usage_start.ru_stime.tv_sec + (usage_start.ru_stime.tv_usec / 1e6);
    double sys_end_t  = usage_end.ru_stime.tv_sec   + (usage_end.ru_stime.tv_usec / 1e6);

    double total_cpu_time = (user_end_t - user_start) + (sys_end_t - sys_start);

    // Real/wall time
    std::chrono::duration<double> real_elapsed = real_end - real_start;
    double real_time = real_elapsed.count();

    // CPU usage % (relative to one CPU core)
    double cpu_usage_percent = (total_cpu_time / real_time) * 100.0;
    printf("\nAvg CPU (%): %.1f\n", cpu_usage_percent);
	sm_st.end();
	return 1;
}
