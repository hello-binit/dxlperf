
#include "Transport.h"

/*

Loop protocol is:

There are two SHM channels
* Streaming: Do streaming RPC requests. For each cycle, reply with a pull_status from the uC
* One-Shot: Do a single RPC request

ONE-SHOT
1. RPC Data is push into SHM from Python side. 
2. Loop is signaled that data is ready
3. Data is COBBS encode 

The  packet is:

Data can be up to X bytes.

Data is manually packed / unpacked into dictionaries (Python) and C-structs (Arduino).
Care should be taken that the pack/unpack size and types are consistent between the two.
This is not automated.
*/

bool Transport::startup(const char* serialPort)
{
    //hardcode baudrate for now as using USB->serial
    valid_port= begin(B115200,serialPort);
    return valid_port;
}

bool Transport::sendFramedData(uint8_t * frame_buf, size_t nb_frame)
{
    /*
    Encode a frame and transmit it
    frame_buf: un-encoded frame to transmit, size >= RPC_MAX_FRAME_SIZE
    nb_frame: number of bytes in frame_buf
    */
    
    crc.clearCrc();
    uint16_t value = crc.Modbus(frame_buf,0,nb_frame);
    frame_buf[nb_frame++]=(value>>8)&0xff;
    frame_buf[nb_frame++]= value&0xff;

    uint8_t encoded_buf[RPC_MAX_FRAME_SIZE]; 
    int nb = cobs.encode(frame_buf,nb_frame,encoded_buf);
    encoded_buf[nb++]=COBBS_PACKET_MARKER;
    writeSCS(encoded_buf, nb);
    wFlushSCS();
}

bool Transport::receiveFramedData(uint8_t * frame_buf, size_t & nb_frame)
{
    /*
    Recieve a frame back and decode:
        frame_buf: buf to decode frame into
        nb_frame: number of bytes in resultant frame_buf
    Return true if coherent frame recieved 
    */

    int fs_sel;
    fd_set fs_read;
	int rvLen = 0;
    struct timeval time;
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec = 0;
    time.tv_usec = IOTimeOut*1000;

    nb_frame=0;
    int nrx=0;
    int nf=0;
    uint8_t tmp_buf[RPC_MAX_FRAME_SIZE]; 
    uint8_t encoded_buf[RPC_MAX_FRAME_SIZE];
    crc_ok=true;
	while(1)
    {
		fs_sel = select(fd+1, &fs_read, NULL, NULL, &time);
		if(fs_sel)
        {
			nrx= read(fd, tmp_buf, RPC_MAX_FRAME_SIZE);
            if (nrx>RPC_MAX_FRAME_SIZE)
            {
                printf("receiveFramedData: Bad NRX: %d \n", nrx);
                return false;
            }
            for (int nn=0;nn<nrx;nn++)
            {
                if (tmp_buf[nn]==COBBS_PACKET_MARKER) //End of Frame, process it
                {    
                    nb_frame=cobs.decode(encoded_buf,nf,frame_buf);
                    crc.clearCrc();
                    uint16_t crc1 = (frame_buf[nb_frame-2]<<8)+frame_buf[nb_frame-1];
                    uint16_t crc2 = crc.Modbus(frame_buf,0,nb_frame-2);
                    if (crc1!=crc2)
                    {
                        crc_ok=false;
                        printf("receiveFramedData: Bad CRC: %d | %d\n", crc1,crc2);
                        return false;
                    }
                    nb_frame=nb_frame-2; //drop CRC
                    return true;
                }
                else
                {
                    encoded_buf[nf++]=tmp_buf[nn];
                    if (nf>RPC_MAX_FRAME_SIZE)
                    {
                        printf("receiveFramedData: Failed to get COBBS_PACKET_MARKER\n");
                        return false;
                    }
                }
            }
        }
    }
}


bool Transport::handle_push_ack_v1(bool crc, size_t nr, int ack_code)
{
/*
Utility function for code readability
*/
/*if self.dbg_on:
    if nr:
        self.dbg_buf = self.dbg_buf + 'Framer rcvd on RPC_V1_PUSH_ACK CRC: ' + str(crc) + ' NR: ' + str(nr) + \
                       ' B0: ' + str(ack_code) + ' Expected B0: ' + str(RPC_V1_PUSH_ACK) + ':' + self.port_name
    else:
        self.dbg_buf = self.dbg_buf + 'Framer rcvd 0 bytes on RPC_PUSH_ACK'*/
    if (crc != 1)
    {
        printf("Transport CRC Error on RPC_V1_PUSH_ACK\n");// {0} {1} {2}'.format(crc, nr, ack_code))
        return false;
    }
    
    if (ack_code != RPC_V1_PUSH_ACK)
    {
        printf("Transport RX Error on RPC_V1_PUSH_ACK\n");// {0} {1} {2}'.format(crc, nr, ack_code))
        return false;
    }
    return true;
}


bool Transport::doPushTransaction(uint8_t * rpc_data, size_t nb_rpc_data,
    void (*rpc_callback)(uint8_t * rpc_reply, size_t nb_rpc_reply))
{
    /*
    Parameters
    ----------
    rpc_data: Buffer of RPC data to transmit to uC
    nb_rpc_data: number of bytes in buffer
    rpc_callback: to be called upon success
    Returns: True/False if successful
    -------
    Push command data
    Take the rpc_data, break into 64 byte encoded frames, and write to serial.
    Recieve back acks for each frame sent

    RPC data looks like:
    rpc_data = [RPC_ID D0, D1,...] (Len 1024 max)

    The rpc_data is then grouped into one or more 59 byte frames.

    For rpc_data<=59 bytes, an RPC send is straigthforward:

    push_frame_0 = [RPC_PUSH_FRAME_LAST, D0,...DN, CRC1 CRC2] (len 62 max, N<=58)

    If the size of data sent is over 59 bytes, then multiple frames are used. If for example, 150 bytes are sent:

    frame_0 = [RPC_PUSH_FRAME_MORE, , D0,...D58, CRC1 CRC2] (len 62 max)
    frame_1 = [RPC_PUSH_FRAME_MORE, D59,...D117, CRC1 CRC2] (len 62 max)
    frame_2 = [RPC_PUSH_FRAME_LAST, D118,...D149, CRC1 CRC2] (len 62 max)

    Before transmission each frame is first Cobbs encoded as:

    [ OverheadByte 62_bytes_max_encoded DelimiterByte] (Len 64 max)

    A push transaction does not return status data back. It returns a RPC_PUSH_x_ACK to acknowledge that a frame was
    succesfully recieved. It also returns an RPC_ID_ACK for the callback to verify that the correct RPC call was completed.

    reply_frame_0 = [RPC_PUSH_ACK RPC_ID_ACK CRC1 CRC2]

    Finally, reply data is decoded and passed to the rpc_callback
        */

    if (!valid_port)
        return false;
    
    if (nb_rpc_data>RPC_DATA_MAX_BYTES)
    {
        printf("doPushTransaction: RPC data size of %d exceeds max of %d\n",
            nb_rpc_data,RPC_DATA_MAX_BYTES);
        return false;
    }
    transactions++;

    //This will block until  RPC has been completed
    
    //TODO: Bring back exception handling
        

    int n_frames = std::ceil(nb_rpc_data / RPC_V1_FRAME_DATA_MAX_BYTES);
    int widx = 0;
    uint8_t frame_buf_tx[RPC_MAX_FRAME_SIZE];
    uint8_t frame_buf_rx[RPC_MAX_FRAME_SIZE];
    size_t nb_frame_rx;

    for (int fid=0;fid<n_frames;fid++)
    {
        std::memset(frame_buf_tx,0,RPC_MAX_FRAME_SIZE); 
        //Build the Nth frame and transmit
        if (fid == 0 && n_frames == 1)
        frame_buf_tx[0] = RPC_V1_PUSH_FRAME_FIRST_ONLY;
        else if(fid == 0 && n_frames > 1)
        frame_buf_tx[0] = RPC_V1_PUSH_FRAME_FIRST_MORE;
        else if(fid == n_frames - 1)
        frame_buf_tx[0] = RPC_V1_PUSH_FRAME_LAST;
        else
        frame_buf_tx[0] = RPC_V1_PUSH_FRAME_MORE;

        int nb_frame = std::min(RPC_V1_FRAME_DATA_MAX_BYTES, (int)nb_rpc_data - widx);
        memcpy(frame_buf_tx+1, rpc_data+widx,nb_frame);
        widx = widx + nb_frame;
        sendFramedData(frame_buf_tx, nb_frame + 1);

        //Get Ack back
        if(receiveFramedData(frame_buf_rx, nb_frame_rx))
        {
            handle_push_ack_v1(crc_ok, nb_frame_rx,frame_buf_rx[0]);
            if(fid == n_frames - 1) //Got last ack
                (*rpc_callback)(frame_buf_rx+1, nb_frame_rx-1);
        }
    }
    return true;
return false;
}









