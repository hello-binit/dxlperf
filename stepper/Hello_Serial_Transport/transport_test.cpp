#include "Transport.h"

#include <iostream>
#include <csignal>
#include <cmath>
#include "time.h"

#include <chrono>
#include <sys/resource.h>  // for getrusage()
#include <sys/time.h>      // for struct rusage's timeval


void signalHandler(int signum) {
    if (signum == SIGINT) {
		std::cout<<"Terminated!"<<std::endl;
        exit(0);
    }
}

#define RPC_SET_COMMAND  1
#define RPC_REPLY_COMMAND  2
#define RPC_GET_STATUS  3
#define RPC_REPLY_STATUS  4
#define RPC_SET_GAINS  5
#define RPC_REPLY_GAINS  6
#define RPC_LOAD_TEST_PUSH 7
#define RPC_REPLY_LOAD_TEST_PUSH 8
#define RPC_SET_TRIGGER  9
#define RPC_REPLY_SET_TRIGGER 10
#define RPC_SET_ENC_CALIB 11
#define RPC_REPLY_ENC_CALIB 12
#define RPC_READ_GAINS_FROM_FLASH 13
#define RPC_REPLY_READ_GAINS_FROM_FLASH 14
#define RPC_SET_MENU_ON 15
#define RPC_REPLY_MENU_ON 16
#define RPC_GET_STEPPER_BOARD_INFO 17
#define RPC_REPLY_STEPPER_BOARD_INFO 18
#define RPC_SET_MOTION_LIMITS 19
#define RPC_REPLY_MOTION_LIMITS 20
#define RPC_SET_NEXT_TRAJECTORY_SEG 21
#define RPC_REPLY_SET_NEXT_TRAJECTORY_SEG 22
#define RPC_START_NEW_TRAJECTORY 23
#define RPC_REPLY_START_NEW_TRAJECTORY 24
#define RPC_RESET_TRAJECTORY 25
#define RPC_REPLY_RESET_TRAJECTORY 26
#define RPC_READ_TRACE 27
#define RPC_REPLY_READ_TRACE 28
#define RPC_GET_STATUS_AUX  29
#define RPC_REPLY_STATUS_AUX  30
#define RPC_LOAD_TEST_PULL 31
#define RPC_REPLY_LOAD_TEST_PULL 32

void callback_board_info(uint8_t * rpc_reply, size_t nb_rpc_reply)
{
    std::cout<<"----------- Board Info -----------\n";
    for(int i=0;i<(int)nb_rpc_reply;i++)
    {
        std::cout<<"Idx: "<<i<<" | "<<rpc_reply[i]<<"\n";
    }
}

void get_board_info(Transport & t)
{
    uint8_t rpc_msg[] = {RPC_GET_STEPPER_BOARD_INFO};
    t.doPushTransactionV1(rpc_msg, (size_t)1,callback_board_info);
}

void callback_load_test_push(uint8_t * rpc_reply, size_t nb_rpc_reply)
{
    if (rpc_reply[0] != RPC_REPLY_LOAD_TEST_PUSH)
        std::cout<<"Error RPC_REPLY_LOAD_TEST_PUSH "<<rpc_reply[0]<<std::endl;
    //else
    //    std::cout<<"Success: callback_load_test_push"<<std::endl;
}

uint8_t load_test_msg[1025];
bool first_load_test=true;

void do_load_test_push(Transport & t)
{
    load_test_msg[0]=RPC_LOAD_TEST_PUSH;
    if (first_load_test) //Init payload
    {
        first_load_test=false;
        for (int i=1;i<=1024;i++)
            load_test_msg[i]=i%256;
    }
    else
    {
        for (int i=1;i<=1024;i++)
            load_test_msg[i]=(load_test_msg[i]+1)%256;
    }

    t.doPushTransactionV1(load_test_msg, (size_t)1025,callback_load_test_push);
}

void callback_load_test_pull(uint8_t * rpc_reply, size_t nb_rpc_reply)
{
    if (rpc_reply[0] != RPC_REPLY_LOAD_TEST_PULL)
        std::cout<<"Error RPC_REPLY_LOAD_TEST_PULL "<<rpc_reply[0]<<" | "<<nb_rpc_reply<<std::endl;
    else
    {
        bool fail=false;
        for (int i=1;i<=1024;i++)
        {   
            //printf("Idx %d | TX: %d RX %d\n",i,load_test_msg[i],rpc_reply[i]);
            if (rpc_reply[i] != (load_test_msg[i]+1)%256)
            {
                std::cout<<"FAIL"<<std::endl;
                //'callback_load_test_pull bad data'<<rpc_reply[i]<<" |"<< load_test_msg[(i + 1) % 1024];
                fail=true;
            }
        }
        //if(!fail)
        //    std::cout<<"Success at callback_load_test_pull\n";
    }
}

void do_load_test_pull(Transport & t)
{
    uint8_t rpc_msg[] = {RPC_LOAD_TEST_PULL};
    t.doPullTransactionV1(rpc_msg, (size_t)1,callback_load_test_pull);
}

void callback_status_pull(uint8_t * rpc_reply, size_t nb_rpc_reply)
{
    if (rpc_reply[0] != RPC_REPLY_STATUS)
        std::cout<<"Error RPC_REPLY_STATUS "<<rpc_reply[0]<<" | "<<nb_rpc_reply<<std::endl;
    else
    {
        /*std::cout<<"----------- Status-----------\n";
        for(int i=0;i<(int)nb_rpc_reply;i++)
        {
            std::cout<<"Idx: "<<i<<" | "<<rpc_reply[i]<<"\n";
        }*/
    }
}

void do_status_pull(Transport & t)
{
    uint8_t rpc_msg[] = {RPC_GET_STATUS};
    t.doPullTransactionV1(rpc_msg, (size_t)1,callback_status_pull);
}
int main(int argc, char **argv)
{



    Transport t;
    if(!t.startup("/dev/hello-motor-omni-0"))
    {
    std::cout<<"Failed to init Stepper!"<<std::endl;
    return 0;
    }
    //get_board_info(t);

    struct rusage usage_start;
    getrusage(RUSAGE_SELF, &usage_start);
    auto real_start = std::chrono::steady_clock::now();
    signal(SIGINT, signalHandler);
    struct timespec start, end;

    clock_gettime(CLOCK_MONOTONIC, &start);
    for (int i=0;i<100;i++)
    {
        do_status_pull(t);
        //do_load_test_push(t);
        //do_load_test_pull(t);
    }
    clock_gettime(CLOCK_MONOTONIC, &end);

    double elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1E9;
    printf("RATE: %.2f\n", 100 / elapsed_time);




}

