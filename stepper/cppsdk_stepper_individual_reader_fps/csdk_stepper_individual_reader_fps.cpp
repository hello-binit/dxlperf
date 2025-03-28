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

void signalHandler(int signum) {
    if (signum == SIGINT) {
		std::cout<<"Terminated!"<<std::endl;
        sm_st.end();
        exit(0);
    }
}

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
    double vmin=1000;
	float Pos[4];
	float Speed[4];
	float Temper[4];

    for (int t = 0;t < 1000+throwaway_cnt; t++) {
	
        
        clock_gettime(CLOCK_MONOTONIC, &start);

		for(int i=0; i<sizeof(ID)/sizeof(ID[0]); i++){
            if(sm_st.FeedBack(ID[i])!=-1){
                // Conversions here: https://www.feetechrc.com/en/2020-05-13_56655.html
    			Pos[i] = sm_st.ReadPos(ID[i])*2*M_PI/4096.0; // 1 step=2*PI/4096.0 rad, 
    			Speed[i] = sm_st.ReadSpeed(ID[i])*2*M_PI/4096.0; // 1 steps/s=2*PI/4096.0 rads/sec (50 steps/s≈0.732RPM https://www.waveshare.com/wiki/ST3215_Servo)
                Temper[i] = sm_st.ReadTemper(ID[i]); // 1 : 1 degree Celcius

                /*std::cout<<"Motor="<<static_cast<int>(ID[i])<<" ";
    			std::cout<<"Pos="<<Pos[i]<<"rad ";
    			std::cout<<"Speed="<<Speed[i]<<"rad/sec ";
                std::cout<<"Temperature="<<Temper[i]<<"deg ";
                std::cout<<std::endl;*/
    			
    		}else{
                std::cout<<"Motor="<<static_cast<int>(ID[i])<<" ";
    			std::cout<<"read err"<<std::endl;
    		}
        }

        clock_gettime(CLOCK_MONOTONIC, &end);
        for(int i=0; i<sizeof(ID)/sizeof(ID[0]); i++){
                std::cout<<"Motor="<<static_cast<int>(ID[i]);
    			std::cout<<"Pos="<<Pos[i]<<"rad ";
    			std::cout<<"Speed="<<Speed[i]<<"rad/sec ";
                std::cout<<"Temperature="<<Temper[i]<<"deg ";
                std::cout<<std::endl;}

        if (throwaway_cnt > 0) {
            throwaway_cnt -= 1;
            continue;
        }

        double elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1E9;
        printf("%.2f\n", 1 / elapsed_time);
        diff[t] = elapsed_time;
        vmin=fmin(vmin,1/elapsed_time);

        std::cout<<"------------------------------------------------------"<<std::endl;
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

