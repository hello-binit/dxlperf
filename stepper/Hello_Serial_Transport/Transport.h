#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <iostream>

#include "COBS.h"
#include "Crc16.h"
#include "SCSerial.h"


#define RPC_V1_MAX_FRAMES 18  // Required to support 1024 bytes
#define RPC_V1_PUSH_FRAME_FIRST_MORE 201
#define RPC_V1_PUSH_FRAME_FIRST_ONLY 202
#define RPC_V1_PUSH_FRAME_MORE 203
#define RPC_V1_PUSH_FRAME_LAST  204
#define RPC_V1_PUSH_ACK  205
#define RPC_V1_PULL_FRAME_FIRST 206
#define RPC_V1_PULL_FRAME_MORE 207
#define RPC_V1_PULL_FRAME_ACK_MORE  208
#define RPC_V1_PULL_FRAME_ACK_LAST 209
#define RPC_V1_FRAME_DATA_MAX_BYTES  58 //63 - 2 (CRC) - 1 (Cobbs Header) - 1 (FRAME CMD) - 1 (Packet Marker)

#define COBBS_FRAME_SIZE_V1 63 //Was seeing issues when transmitting 64 bytes so limiting to 63. Issue resolved.

//////////////////////////////  Shared Defines ///////////////////////////////////////////////////
#define RPC_DATA_MAX_BYTES  1024
#define RPC_MAX_FRAME_SIZE 64 //Arduino and Linux USB Uart has a 64 byte buffer. When frame is >64 have seen issues.


//RPC_DATA_MAX_BYTES_SAMD51 = 8194 #SAMD51 needs 8KB of space for 2048 floats + rpc ID
#define COBBS_PACKET_MARKER 0

class Transport : public SCSerial{
   public:
      bool startup(const char* serialPort);
      bool doPushTransactionV1(uint8_t * rpc_out, size_t nb_rpc_out, 
         void (*rpc_callback)(uint8_t * rpc_reply, size_t nb_rpc_reply));
      bool doPullTransactionV1(uint8_t * rpc_out, size_t nb_rpc_out, 
         void (*rpc_callback)(uint8_t * rpc_reply, size_t nb_rpc_reply));
      inline Transport()
      {
         transactions=0;
         valid_port=false;
         crc_ok=false;
      }
   private:
      bool sendFramedData(uint8_t * frame, size_t nb_frame);
      bool receiveFramedData(uint8_t * frame_buf, size_t & nb_frame);
      bool handlePushAckV1(bool crc, size_t nr, int ack_code);
      bool handlePullAckV1(bool crc, size_t nr, int ack_code);

      int transactions;
      bool valid_port;
      bool crc_ok;
      Crc16 crc;
      COBS cobs;



         /*int byte_in_cnt=0;
         int byte_out_cnt=0;
         
         uint8_t rx_buffer[RPC_MAX_FRAME_SIZE];
         uint8_t tx_buffer[RPC_MAX_FRAME_SIZE];
         uint8_t frame_in[RPC_MAX_FRAME_SIZE];
         uint8_t payload[RPC_DATA_MAX_BYTES+1];
         uint8_t rpc_in[RPC_DATA_MAX_BYTES+1]; //Can transmit up to 1024 bytes + 1 byte for RPC ID
         uint8_t rpc_out[RPC_DATA_MAX_BYTES+1];
         int num_byte_rpc_out=0;
         int num_byte_rpc_in=0;*/
        

 


    };


