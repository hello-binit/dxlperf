
#include "transport_shm.h"

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

bool TransportSHM::startup(const char* serialPort)
{
    //hardcode baudrate for now as using USB->serial
    valid_port= begin(B115200,serialPort);
    return valid_port;
}

bool TransportSHM::sendFramedData(uint8_t * frame_buf, size_t nb_frame)
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

bool TransportSHM::receiveFramedData(uint8_t * frame_buf, size_t & nb_frame)
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


bool TransportSHM::handle_push_ack_v1(bool crc, size_t nr, int ack_code)
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


bool TransportSHM::doPushTransaction(uint8_t * rpc_data, size_t nb_rpc_data,
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







# ##################### TRANSPORT ####################################


class TransportError(Exception):
    """Base class for exceptions in this module."""
    pass


class SyncTransactionHandler():
    def __init__(self, port_name, ser, logger, lock):
        self.ser = ser
        self.logger = logger
        self.port_name = port_name
        self.empty_frame = arr.array('B', [0] * RPC_MAX_FRAME_SIZE)
        self.status = {'read_error': 0, 'write_error': 0, 'transactions': 0}
        self.version = RPC_TRANSPORT_VERSION_0
        self.timeout = 1 # was .2  # Was .05 but on heavy loads can get starved
        self.packet_marker = 0
        self.lock = lock
        self.dbg_buf = ''
        self.dbg_on = 0

    def get_empty_frame(self):  # Just a fast convience function to create a large array of 'B'
        return self.empty_frame[:]

    def handle_push_ack_v1(self, crc, nr, ack_code):
        """
        Utility function for code readability
        """
        if self.dbg_on:
            if nr:
                self.dbg_buf = self.dbg_buf + 'Framer rcvd on RPC_V1_PUSH_ACK CRC: ' + str(crc) + ' NR: ' + str(nr) + \
                               ' B0: ' + str(ack_code) + ' Expected B0: ' + str(RPC_V1_PUSH_ACK) + ':' + self.port_name
            else:
                self.dbg_buf = self.dbg_buf + 'Framer rcvd 0 bytes on RPC_PUSH_ACK'
        if crc != 1:
            self.logger.error('Transport CRC Error on RPC_V1_PUSH_ACK {0} {1} {2}'.format(crc, nr, ack_code))
            raise TransportError
        if ack_code != RPC_V1_PUSH_ACK:
            self.logger.error('Transport RX Error on RPC_V1_PUSH_ACK {0} {1} {2}'.format(crc, nr, ack_code))
            raise TransportError

    def handle_pull_ack_v1(self, crc, nr, ack_code):
        """
        Utility function for code readability
        """
        if self.dbg_on:
            if nr:
                self.dbg_buf = self.dbg_buf + 'Framer rcvd on RPC_PULL_ACK CRC: ' + str(crc) + ' NR: ' + str(nr) + \
                               ' B0: ' + str(ack_code) + ' Expected B0: ' + str(RPC_V1_PULL_FRAME_ACK_MORE) + \
                               ' OR B0: ' + str(RPC_V1_PULL_FRAME_ACK_LAST) + ':' + self.port_name
            else:
                self.dbg_buf = self.dbg_buf + 'Framer rcvd 0 bytes on RPC_PULL_ACK'
        if crc != 1:
            self.logger.error('Transport CRC Error on RPC_PUSH_ACK {0} {1} {2}'.format(crc, nr, ack_code))
            raise TransportError
        if ack_code != RPC_V1_PULL_FRAME_ACK_MORE and ack_code != RPC_V1_PULL_FRAME_ACK_LAST:
            self.logger.error('Transport RX Error on RPC_PUSH_ACK {0} {1} {2}'.format(crc, nr, ack_code))
            raise TransportError

    def sendFramedData(self, data, size):
        framer = cobbs_framing.CobbsFraming()
        encoded_data = framer.encode_data(data[:size])
        self.ser.write(encoded_data)

    def receiveFramedData(self):
        t_start = time.time()
        rx_buffer = []
        framer = cobbs_framing.CobbsFraming()
        while ((time.time() - t_start) < self.timeout):
            nn = self.ser.inWaiting()
            if (nn > 0):
                rbuf = self.ser.read(nn)
                nu = 0
                for byte_in in rbuf:
                    nu = nu + 1
                    if byte_in == self.packet_marker:
                        crc_ok, nr, decoded_data = framer.decode_data(rx_buffer)  # [:-1])
                        return crc_ok, nr, decoded_data
                    else:
                        rx_buffer.append(byte_in)
            else:
                time.sleep(.00001)
        return 0, 0, arr.array('B', [])

    def receiveFramedData2(self):
        framer = cobbs_framing.CobbsFraming()
        rbuf = self.ser.read_until(terminator=b'\x00', size=RPC_MAX_FRAME_SIZE)
        if len(rbuf) == 0 or rbuf[-1] != 0:
            self.logger.warn('Warning: Transport invalid read %d bytes during _recv_framed_data' % len(rbuf))
            raise TransportError
        crc_ok, nr, decoded_data = framer.decode_data(rbuf[:-1])
        return crc_ok, nr, decoded_data

    def do_rpc(self, push, payload, reply_callback, exiting=False):
        if not self.ser:
            return
        self.status['transactions'] += 1
        if exiting:
            time.sleep(0.1)  # May have been a hard exit, give time for bad data to land, remove, do final RPC
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
        # This will block until all RPCs have been completed
        try:
            # Now run RPC calls
            if push:
                if self.version == RPC_TRANSPORT_VERSION_0:
                    self.do_transaction_v0(payload, reply_callback)
                elif self.version == RPC_TRANSPORT_VERSION_1:
                    self.do_push_transaction_v1(payload, reply_callback)
            else:
                if self.version == RPC_TRANSPORT_VERSION_0:
                    self.do_transaction_v0(payload, reply_callback)
                elif self.version == RPC_TRANSPORT_VERSION_1:
                    self.do_pull_transaction_v1(payload, reply_callback)
        except IOError as e:
            print("IOError({0}): {1}".format(e.errno, e.strerror))
            self.status['read_error'] += 1
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            print("SerialException({0}): {1}".format(e.errno, e.strerror))

    def do_push_transaction_v1(self, rpc_data, rpc_callback):
        """
                Parameters
        ----------
        rpc_data: Buffer of data to transmit in RPC
        rpc_callback: Call back to process RPC reply data
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
        """
        n_frames = math.ceil(len(rpc_data) / RPC_V1_FRAME_DATA_MAX_BYTES)
        widx = 0
        frame_buf_out = self.get_empty_frame()

        try:
            for fid in range(n_frames):
                # Build the Nth frame and transmit
                if fid == 0 and n_frames == 1:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_FIRST_ONLY
                elif fid == 0 and n_frames > 1:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_FIRST_MORE
                elif fid == n_frames - 1:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_LAST
                else:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_MORE
                nb_frame = min(RPC_V1_FRAME_DATA_MAX_BYTES, len(rpc_data) - widx)
                frame_buf_out[1:nb_frame + 1] = rpc_data[widx:widx + nb_frame]
                widx = widx + nb_frame
                self.sendFramedData(frame_buf_out, nb_frame + 1)
                # Get Ack back
                crc_ok, nr, decoded_data = self.receiveFramedData()
                self.handle_push_ack_v1(crc_ok, nr, decoded_data[0])
                if fid == n_frames - 1:
                    rpc_callback(decoded_data[1:nr])
            return True
        except TransportError as e:
            if self.dbg_on:
                print('---- Debug Exception')
                print(self.dbg_buf)
            self.status['read_error'] += 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.logger.error("TransportError: %s : %s" % (self.port_name, str(e)))
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            self.ser = None
            self.logger.error("SerialTimeoutException: %s : %s" % (self.port_name, str(e)))
        except serial.SerialException as e:
            self.logger.error("SerialException: %s : %s" % (self.port_name, str(e)))
            self.ser = None
        except TypeError as e:
            self.logger.error("TypeError: %s : %s" % (self.port_name, str(e)))
            self.ser = None
        return False

    def do_pull_transaction_v1(self, rpc_data, rpc_callback):
        """
                Parameters
        ----------
        rpc_data: Buffer of data to transmit in RPC
        rpc_callback: Call back to process RPC reply data
        Returns: True/False if successful
        -------
        This pulls status data from the device. The frame layout is analogous to do_push_transaction.
        However, here we send down only a pull request (single frame with an RPC_ID).
        We get back one or more frames of status data which is then decoded and passed to the callback.
        """
        frame_buf_out = self.get_empty_frame()
        try:
            # First initiate a pull transaction
            frame_buf_out[0] = RPC_V1_PULL_FRAME_FIRST
            nb_frame = min(RPC_V1_FRAME_DATA_MAX_BYTES, len(rpc_data))
            frame_buf_out[1:nb_frame + 1] = rpc_data[0:nb_frame]
            self.sendFramedData(frame_buf_out, nb_frame + 1)

            reply = arr.array('B')

            # Next read out the N reply frames (up to 18, if no ACK_LAST, then error
            for i in range(RPC_V1_MAX_FRAMES):

                crc, nr, decoded_data = self.receiveFramedData()
                self.handle_pull_ack_v1(crc, nr, decoded_data[0])
                reply = reply + decoded_data[1:nr]
                if decoded_data[0] == RPC_V1_PULL_FRAME_ACK_LAST:  # No more frames to request
                    rpc_callback(reply)
                    return True
                elif decoded_data[0] == RPC_V1_PULL_FRAME_ACK_MORE:  # More frames to request
                    frame_buf_out[0] = RPC_V1_PULL_FRAME_MORE
                    nb_frame = min(RPC_V1_FRAME_DATA_MAX_BYTES, len(rpc_data))
                    frame_buf_out[1:nb_frame + 1] = rpc_data[0:nb_frame]
                    self.sendFramedData(frame_buf_out, nb_frame + 1)
                else:
                    raise TransportError
            self.logger.error("In do_pull_transaction. Failed to get RPC_V1_PULL_FRAME_ACK_LAST")
            raise TransportError
        except TransportError as e:
            if self.dbg_on:
                print('---- Debug Exception')
                print(self.dbg_buf)
            self.status['read_error'] += 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.logger.error("TransportError: %s : %s" % (self.port_name, str(e)))
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            self.ser = None
            self.logger.error("SerialTimeoutException: %s : %s" % (self.port_name, str(e)))
        except serial.SerialException as e:
            self.logger.error("SerialException: %s : %s" % (self.port_name, str(e)))
            self.ser = None
        except TypeError as e:
            self.logger.error("TypeError: %s : %s" % (self.port_name, str(e)))
            self.ser = None
        return False

    def do_transaction_v0(self, rpc, rpc_callback):  # Handle a single RPC transaction
        frame_buf = arr.array('B', [0] * (COBBS_FRAME_SIZE_V0))
        try:
            if self.dbg_on:
                dbg_buf = self.dbg_buf + '--------------- New RPC -------------------------\n'
            ########## Initiate new RPC
            frame_buf[0] = RPC_V0_START_NEW_RPC
            self.sendFramedData(frame_buf, 1)
            if self.dbg_on:
                self.dbg_buf = self.dbg_buf + 'Framer sent RPC_V0_START_NEW_RPC\n'
            crc, nr, decoded_data = self.receiveFramedData()
            if self.dbg_on:
                if nr:
                    self.dbg_buf = self.dbg_buf + 'Framer rcvd on RPC_V0_ACK_NEW_RPC CRC: ' + str(crc) + ' NR: ' + str(
                        nr) + ' B0: ' + str(decoded_data[0]) + ' Expected B0: ' + str(
                        RPC_V0_ACK_NEW_RPC) + ':' + self.port_name
                else:
                    self.dbg_buf = self.dbg_buf + 'Framer rcvd 0 bytes on RPC_V0_ACK_NEW_RPC'

            if crc != 1 or decoded_data[0] != RPC_V0_ACK_NEW_RPC:
                self.logger.error(
                    'Transport RX Error on RPC_V0_ACK_NEW_RPC {0} {1} {2}'.format(crc, nr, decoded_data[0]))
                raise TransportError
            # if self.dbg_on:
            #    print('New RPC initiated, len',len(rpc))
            ########### Send all blocks
            ntx = 0
            while ntx < len(rpc):
                nb = min(len(rpc) - ntx, RPC_V0_BLOCK_SIZE)  # Num bytes to send
                b = rpc[ntx:ntx + nb]
                ntx = ntx + nb
                if ntx == len(rpc):  # Last block
                    frame_buf[0] = RPC_V0_SEND_BLOCK_LAST
                    frame_buf[1:len(b) + 1] = b
                    # if self.dbg_on:
                    #    print('Sending last block',ntx)
                    self.sendFramedData(frame_buf, nb + 1)
                    if self.dbg_on:
                        self.dbg_buf = self.dbg_buf + 'Framer sent RPC_V0_SEND_BLOCK_LAST\n'
                    # if self.dbg_on:
                    #    print('Getting last block ack',ntx)

                    crc, nr, decoded_data = self.receiveFramedData()
                    if self.dbg_on:
                        if nr:
                            self.dbg_buf = self.dbg_buf + 'Framer rcvd on RPC_V0_SEND_BLOCK_LAST CRC: ' + str(
                                crc) + ' NR: ' + str(nr) + ' B0: ' + str(decoded_data[0]) + ' Expected B0: ' + str(
                                RPC_V0_ACK_SEND_BLOCK_LAST) + ':' + self.port_name + '\n'
                        else:
                            self.dbg_buf = self.dbg_buf + 'Framer rcvd 0 bytes on RPC_V0_SEND_BLOCK_LAST'
                    # if self.dbg_on:
                    #    print('Last block ack rcvd',ntx)
                    if crc != 1 or decoded_data[0] != RPC_V0_ACK_SEND_BLOCK_LAST:
                        self.logger.error('Transport RX Error on RPC_V0_ACK_SEND_BLOCK_LAST {0} {1} {2}'.format(crc, nr,
                                                                                                                decoded_data[
                                                                                                                    0]))
                        raise TransportError
                else:
                    frame_buf[0] = RPC_V0_SEND_BLOCK_MORE
                    frame_buf[1:len(b) + 1] = b
                    # if self.dbg_on:
                    #    print('Sending next block',ntx)
                    self.sendFramedData(frame_buf, nb + 1)
                    if self.dbg_on:
                        self.dbg_buf = self.dbg_buf + 'Framer sent RPC_V0_SEND_BLOCK_MORE\n'
                    # if self.dbg_on:
                    #    print('Sent next block',ntx)
                    crc, nr, decoded_data = self.receiveFramedData()
                    if self.dbg_on:
                        if nr:
                            self.dbg_buf = self.dbg_buf + 'Framer rcvd on RPC_V0_SEND_BLOCK_MORE CRC: ' + str(
                                crc) + ' NR: ' + str(nr) + ' B0: ' + str(decoded_data[0]) + ' Expected B0: ' + str(
                                RPC_V0_ACK_SEND_BLOCK_MORE) + ':' + self.port_name + '\n'
                        else:
                            self.dbg_buf = self.dbg_buf + 'Framer rcvd 0 bytes on RPC_V0_SEND_BLOCK_MORE'
                    if crc != 1 or decoded_data[0] != RPC_V0_ACK_SEND_BLOCK_MORE:
                        self.logger.error('Transport RX Error on RPC_V0_ACK_SEND_BLOCK_MORE {0} {1} {2}'.format(crc, nr,
                                                                                                                decoded_data[
                                                                                                                    0]))
                        raise TransportError
            ########### Receive all blocks
            reply = arr.array('B')
            # if self.dbg_on:
            #    print('Receiving RPC reply')
            while True:
                frame_buf[0] = RPC_V0_GET_BLOCK
                # if self.dbg_on:
                #    print('Block requested')
                self.sendFramedData(frame_buf, 1)
                crc, nr, decoded_data = self.receiveFramedData()
                # if self.dbg_on:
                #    print('Block request success')
                if crc != 1 or not (
                        decoded_data[0] == RPC_V0_ACK_GET_BLOCK_MORE or decoded_data[0] == RPC_V0_ACK_GET_BLOCK_LAST):
                    self.logger.error(
                        'Transport RX Error on RPC_V0_GET_BLOCK {0} {1} {2}'.format(crc, nr, decoded_data[0]))
                    raise TransportError
                reply = reply + decoded_data[1:nr]

                if decoded_data[0] == RPC_V0_ACK_GET_BLOCK_LAST:
                    break
            # Now process the reply
            # if self.dbg_on:
            #    print('Got reply',len(reply))
            # print('---------------------- RPC complete, elapsed time------------------:',time.time()-ts)
            rpc_callback(reply)
        except TransportError as e:
            if self.dbg_on:
                print('---- Debug Exception')
                print(self.dbg_buf)
            self.status['read_error'] += 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.logger.error("TransportError: %s : %s" % (self.port_name, str(e)))
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            self.ser = None
            self.logger.error("SerialTimeoutException: %s : %s" % (self.port_name, str(e)))
        except serial.SerialException as e:
            self.logger.error("SerialException: %s : %s" % (self.port_name, str(e)))
            self.ser = None
        # except TypeError as e:
        #     self.logger.error("TypeError: %s : %s" % (self.port_name, str(e)))
        #     self.ser=None


class AsyncTransactionHandler(SyncTransactionHandler):
    def __init__(self, port_name, ser, logger, lock):
        SyncTransactionHandler.__init__(self, port_name, ser, logger, lock)

    async def sendFramedData(self, data, size):
        framer = cobbs_framing.CobbsFraming()
        encoded_data = framer.encode_data(data[:size])
        await self.ser.write_async(encoded_data)

    async def receiveFramedData(self):
        t_start = time.time()
        rx_buffer = []
        framer = cobbs_framing.CobbsFraming()
        while ((time.time() - t_start) < self.timeout):
            nn = self.ser.inWaiting()
            if (nn > 0):
                rbuf = await self.ser.read_async(nn)
                nu = 0
                for byte_in in rbuf:
                    nu = nu + 1
                    if byte_in == self.packet_marker:
                        crc_ok, nr, decoded_data = framer.decode_data(rx_buffer)  # [:-1])
                        # self.logger.info(f"AsyncTransaction Wait: {1000*(time.time() - t_start)}ms")
                        return crc_ok, nr, decoded_data
                    else:
                        rx_buffer.append(byte_in)
            else:
                time.sleep(.00001)
        self.logger.error(f"Async-Transaction Timeout.")
        return 0, 0, arr.array('B', [])

    async def do_rpc(self, push, payload, reply_callback, exiting=False):
        if not self.ser:
            return
        self.status['transactions'] += 1
        # if exiting:
        #     time.sleep(0.1) #May have been a hard exit, give time for bad data to land, remove, do final RPC
        #     self.ser.reset_output_buffer()
        #     self.ser.reset_input_buffer()
        # This will block until all RPCs have been completed
        try:
            # Now run RPC calls
            if push:
                if self.version == RPC_TRANSPORT_VERSION_0:
                    await self.do_transaction_v0(payload, reply_callback)
                elif self.version == RPC_TRANSPORT_VERSION_1:
                    await self.do_push_transaction_v1(payload, reply_callback)
            else:
                if self.version == RPC_TRANSPORT_VERSION_0:
                    await self.do_transaction_v0(payload, reply_callback)
                elif self.version == RPC_TRANSPORT_VERSION_1:
                    await self.do_pull_transaction_v1(payload, reply_callback)
        except IOError as e:
            print("IOError({0}): {1}".format(e.errno, e.strerror))
            self.status['read_error'] += 1
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            print("SerialException({0}): {1}".format(e.errno, e.strerror))

    async def do_pull_transaction_v1(self, rpc_data, rpc_callback):
        """
                Parameters
        ----------
        rpc_data: Buffer of data to transmit in RPC
        rpc_callback: Call back to process RPC reply data
        Returns: True/False if successful
        -------
        This pulls status data from the device. The frame layout is analogous to do_push_transaction.
        However, here we send down only a pull request (single frame with an RPC_ID).
        We get back one or more frames of status data which is then decoded and passed to the callback.
        """
        frame_buf_out = self.get_empty_frame()

        try:
            # First initiate a pull transaction
            frame_buf_out[0] = RPC_V1_PULL_FRAME_FIRST
            nb_frame = min(RPC_V1_FRAME_DATA_MAX_BYTES, len(rpc_data))
            frame_buf_out[1:nb_frame + 1] = rpc_data[0:nb_frame]
            await self.sendFramedData(frame_buf_out, nb_frame + 1)
            reply = arr.array('B')

            # Next read out the N reply frames (up to 18, if no ACK_LAST, then error
            for i in range(RPC_V1_MAX_FRAMES):
                crc, nr, decoded_data = await self.receiveFramedData()
                self.handle_pull_ack_v1(crc, nr, decoded_data[0])
                reply = reply + decoded_data[1:nr]
                if decoded_data[0] == RPC_V1_PULL_FRAME_ACK_LAST:  # No more frames to request
                    rpc_callback(reply)
                    return True
                elif decoded_data[0] == RPC_V1_PULL_FRAME_ACK_MORE:  # More frames to request
                    frame_buf_out[0] = RPC_V1_PULL_FRAME_MORE
                    nb_frame = min(RPC_V1_FRAME_DATA_MAX_BYTES, len(rpc_data))
                    frame_buf_out[1:nb_frame + 1] = rpc_data[0:nb_frame]
                    await self.sendFramedData(frame_buf_out, nb_frame + 1)
                else:
                    raise TransportError
            self.logger.error("In do_pull_transaction. Failed to get RPC_V1_PULL_FRAME_ACK_LAST")
            raise TransportError
        except TransportError as e:
            if self.dbg_on:
                print('---- Debug Exception')
                print(self.dbg_buf)
            self.status['read_error'] += 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.logger.error("TransportError: %s : %s" % (self.port_name, str(e)))
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            self.ser = None
            self.logger.error("SerialTimeoutException: %s : %s" % (self.port_name, str(e)))
        except serial.SerialException as e:
            self.logger.error("SerialException: %s : %s" % (self.port_name, str(e)))
            self.ser = None
        except TypeError as e:
            self.logger.error("TypeError: %s : %s" % (self.port_name, str(e)))
            self.ser = None
        return False

    async def do_push_transaction_v1(self, rpc_data, rpc_callback):
        """
                Parameters
        ----------
        rpc_data: Buffer of data to transmit in RPC
        rpc_callback: Call back to process RPC reply data
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
        """

        n_frames = math.ceil(len(rpc_data) / RPC_V1_FRAME_DATA_MAX_BYTES)
        widx = 0
        frame_buf_in = self.get_empty_frame()
        frame_buf_out = self.get_empty_frame()

        try:
            for fid in range(n_frames):
                # Build the Nth frame and transmit
                if fid == 0 and n_frames == 1:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_FIRST_ONLY
                elif fid == 0 and n_frames > 1:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_FIRST_MORE
                elif fid == n_frames - 1:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_LAST
                else:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_MORE
                nb_frame = min(RPC_V1_FRAME_DATA_MAX_BYTES, len(rpc_data) - widx)
                frame_buf_out[1:nb_frame + 1] = rpc_data[widx:widx + nb_frame]
                widx = widx + nb_frame
                await self.sendFramedData(frame_buf_out, nb_frame + 1)
                # Get Ack back
                frame_buf_in[0] = 0  # Remove stale data
                crc, nr = await self.receiveFramedData()
                self.handle_push_ack_v1(crc, nr, frame_buf_in[0])
                if frame_buf_in[0] != RPC_V1_PUSH_ACK:
                    raise TransportError("Byte 0 not RPC_V1_PUSH_ACK")
                if fid == n_frames - 1:
                    rpc_callback(frame_buf_in[1:nr])
            return True
        except TransportError as e:
            if self.dbg_on:
                print('---- Debug Exception')
                print(self.dbg_buf)
            self.status['read_error'] += 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.logger.error("TransportError: %s : %s" % (self.port_name, str(e)))
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            self.ser = None
            self.logger.error("SerialTimeoutException: %s : %s" % (self.port_name, str(e)))
        except serial.SerialException as e:
            self.logger.error("SerialException: %s : %s" % (self.port_name, str(e)))
            self.ser = None
        except TypeError as e:
            self.logger.error("TypeError: %s : %s" % (self.port_name, str(e)))
            self.ser = None
        return False

    async def do_transaction_v0(self, rpc, rpc_callback):  # Handle a single RPC transaction
        frame_buf = arr.array('B', [0] * (COBBS_FRAME_SIZE_V0))
        try:
            if self.dbg_on:
                self.dbg_buf = self.dbg_buf + '--------------- New RPC -------------------------\n'
            ########## Initiate new RPC
            frame_buf[0] = RPC_V0_START_NEW_RPC
            await self.sendFramedData(frame_buf, 1)
            if self.dbg_on:
                self.dbg_buf = self.dbg_buf + 'Framer sent RPC_V0_START_NEW_RPC\n'
            crc_ok, nr, decoded_data = await self.receiveFramedData()
            if self.dbg_on:
                if nr:
                    self.dbg_buf = self.dbg_buf + 'Framer rcvd on RPC_V0_ACK_NEW_RPC CRC: ' + str(
                        crc_ok) + ' NR: ' + str(nr) + ' B0: ' + str(decoded_data[0]) + ' Expected B0: ' + str(
                        RPC_V0_ACK_NEW_RPC) + ':' + self.port_name
                else:
                    self.dbg_buf = self.dbg_buf + 'Framer rcvd 0 bytes on RPC_V0_ACK_NEW_RPC'

            if crc_ok != 1 or decoded_data[0] != RPC_V0_ACK_NEW_RPC:
                self.logger.error(
                    'Transport RX Error on RPC_V0_ACK_NEW_RPC {0} {1} {2}'.format(crc_ok, nr, decoded_data[0]))
                raise TransportError

            # if self.dbg_on:
            #    print('New RPC initiated, len',len(rpc))
            ########### Send all blocks
            ntx = 0
            while ntx < len(rpc):
                nb = min(len(rpc) - ntx, RPC_V0_BLOCK_SIZE)  # Num bytes to send
                b = rpc[ntx:ntx + nb]
                ntx = ntx + nb
                if ntx == len(rpc):  # Last block
                    frame_buf[0] = RPC_V0_SEND_BLOCK_LAST
                    frame_buf[1:len(b) + 1] = b
                    # if self.dbg_on:
                    #    print('Sending last block',ntx)
                    await self.sendFramedData(frame_buf, nb + 1)
                    if self.dbg_on:
                        self.dbg_buf = self.dbg_buf + 'Framer sent RPC_V0_SEND_BLOCK_LAST\n'
                    # if self.dbg_on:
                    #    print('Getting last block ack',ntx)

                    crc_ok, nr, decoded_data = await self.receiveFramedData()

                    if self.dbg_on:
                        if nr:
                            self.dbg_buf = self.dbg_buf + 'Framer rcvd on RPC_V0_SEND_BLOCK_LAST CRC: ' + str(
                                crc_ok) + ' NR: ' + str(nr) + ' B0: ' + str(decoded_data[0]) + ' Expected B0: ' + str(
                                RPC_V0_ACK_SEND_BLOCK_LAST) + ':' + self.port_name + '\n'
                        else:
                            self.dbg_buf = self.dbg_buf + 'Framer rcvd 0 bytes on RPC_V0_SEND_BLOCK_LAST'
                    # if self.dbg_on:
                    #    print('Last block ack rcvd',ntx)
                    if crc_ok != 1 or decoded_data[0] != RPC_V0_ACK_SEND_BLOCK_LAST:
                        self.logger.error(
                            'Transport RX Error on RPC_V0_ACK_SEND_BLOCK_LAST {0} {1} {2}'.format(crc_ok, nr,
                                                                                                  decoded_data[0]))
                        raise TransportError
                else:
                    frame_buf[0] = RPC_V0_SEND_BLOCK_MORE
                    frame_buf[1:len(b) + 1] = b
                    # if self.dbg_on:
                    #    print('Sending next block',ntx)
                    await self.sendFramedData(frame_buf, nb + 1)
                    if self.dbg_on:
                        self.dbg_buf = self.dbg_buf + 'Framer sent RPC_V0_SEND_BLOCK_MORE\n'
                    # if self.dbg_on:
                    #    print('Sent next block',ntx)
                    crc_ok, nr, decoded_data = await self.receiveFramedData()
                    if self.dbg_on:
                        if nr:
                            self.dbg_buf = self.dbg_buf + 'Framer rcvd on RPC_V0_SEND_BLOCK_MORE CRC: ' + str(
                                crc_ok) + ' NR: ' + str(nr) + ' B0: ' + str(decoded_data[0]) + ' Expected B0: ' + str(
                                RPC_V0_ACK_SEND_BLOCK_MORE) + ':' + self.port_name + '\n'
                        else:
                            self.dbg_buf = self.dbg_buf + 'Framer rcvd 0 bytes on RPC_V0_SEND_BLOCK_MORE'
                    if crc_ok != 1 or decoded_data[0] != RPC_V0_ACK_SEND_BLOCK_MORE:
                        self.logger.error(
                            'Transport RX Error on RPC_V0_ACK_SEND_BLOCK_MORE {0} {1} {2}'.format(crc_ok, nr,
                                                                                                  frame_buf[0]))
                        raise TransportError
            ########### Receive all blocks
            reply = arr.array('B')
            # if self.dbg_on:
            #    print('Receiving RPC reply')
            while True:
                frame_buf[0] = RPC_V0_GET_BLOCK
                # if self.dbg_on:
                #    print('Block requested')
                await self.sendFramedData(frame_buf, 1)
                crc_ok, nr, decoded_data = await self.receiveFramedData()
                # if self.dbg_on:
                #    print('Block request success')
                if crc_ok != 1 or not (
                        decoded_data[0] == RPC_V0_ACK_GET_BLOCK_MORE or decoded_data[0] == RPC_V0_ACK_GET_BLOCK_LAST):
                    self.logger.error(
                        'Transport RX Error on RPC_V0_GET_BLOCK {0} {1} {2}'.format(crc_ok, nr, decoded_data[0]))
                    raise TransportError
                reply = reply + decoded_data[1:nr]

                if decoded_data[0] == RPC_V0_ACK_GET_BLOCK_LAST:
                    break
            # Now process the reply
            # if self.dbg_on:
            #    print('Got reply',len(reply))
            # print('---------------------- RPC complete, elapsed time------------------:',time.time()-ts)
            rpc_callback(reply)
        except TransportError as e:
            if self.dbg_on:
                print('---- Debug Exception')
                print(self.dbg_buf)
            self.status['read_error'] += 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.logger.error("TransportError: %s : %s" % (self.port_name, str(e)))
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            self.ser = None
            self.logger.error("SerialTimeoutException: %s : %s" % (self.port_name, str(e)))
        except serial.SerialException as e:
            self.logger.error("SerialException: %s : %s" % (self.port_name, str(e)))
            self.ser = None
        except TypeError as e:
            self.logger.error("TypeError: %s : %s" % (self.port_name, str(e)))
            self.ser = None


class Transport():
    """
    Handle serial communications to a Hello Robot USB device using pySerial and asyncio

    Aioserial supports the standard  pySerial interface as well as async versions of the pySerial interface.
    This enables Transport to support both synchronous and asynchromous calls.
    Devices can use standard pySerial for non-timing critical transactions.
    They can use the asyncio interfaces for timing critical transactions where blocking on the RPC call is not desirable
    """

    def __init__(self, usb, logger=logging.getLogger()):
        self.port_name = usb
        self.logger = logger
        self.lock = threading.Lock()
        self.status = {}
        self.empty_payload = arr.array('B', [0] * (RPC_DATA_MAX_BYTES_SAMD51 + 1))  # RPC ID + 1024 bytes of data
        self.version = RPC_TRANSPORT_VERSION_0

    def startup(self):
        try:
            self.logger.debug('Starting TransportConnection on: ' + self.port_name)
            self.ser = aioserial.AioSerial(self.port_name, write_timeout=1.0, timeout=1.0)
            # self.ser = serial.Serial(self.port_name,write_timeout=1.0)  # PosixPollSerial(self.usb)#Serial(self.usb)# 115200)  # , write_timeout=1.0)  # Baud not important since USB comms
            self.async_handler = AsyncTransactionHandler(port_name=self.port_name, ser=self.ser, logger=self.logger,
                                                         lock=self.lock)
            self.sync_handler = SyncTransactionHandler(port_name=self.port_name, ser=self.ser, logger=self.logger,
                                                       lock=self.lock)
            self.status['async'] = self.async_handler.status
            self.status['sync'] = self.sync_handler.status
            if self.ser.isOpen():
                try:
                    fcntl.flock(self.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except IOError:
                    self.logger.error(
                        'Port %s is busy. Check if another Stretch Body process is already running' % self.port_name)
                    self.ser.close()
                    self.ser = None
        except serial.SerialException as e:
            self.logger.error("SerialException({0}): {1}".format(e.errno, e.strerror))
            self.ser = None
        if self.ser == None:
            self.logger.warning('Unable to open serial port for device %s' % self.port_name)
        return self.ser is not None  # return if hardware connection valid

    def stop(self):
        if self.ser:
            self.logger.debug('Shutting down TransportConnection on: ' + self.port_name)
            try:
                fcntl.flock(self.ser.fileno(), fcntl.LOCK_UN | fcntl.LOCK_NB)
            except IOError:
                self.logger.error(
                    'Port %s is busy. Check if another Stretch Body process is already running' % self.port_name)
            self.ser.close()
            self.ser = None

    def get_empty_payload(self):  # Just a fast convience function to create a large array of 'B'
        return self.empty_payload[:]

    def configure_version(self, firmware_version):
        """
        Starting with Stepper/Wacc/Pimu firmware v0.4.0 a faster version (V1) of the transport layer is supported
        Check here if can run the Transport in V1 mode
        """
        xl = firmware_version.split('.')
        if len(xl) != 4:
            raise Exception('Invalid firmware version len')
        device = xl[0]
        if not (device == 'Stepper' or device == 'Wacc' or device == 'Pimu'):
            raise Exception('Invalid device name ')
        major = int(xl[1][1:])
        minor = int(xl[2])
        if major == 0 and minor <= 3:
            self.set_version(RPC_TRANSPORT_VERSION_0)
        else:
            self.set_version(RPC_TRANSPORT_VERSION_1)

    def set_version(self, v):
        # print('VERSION',v,self.port_name)
        if not v == 0 and not v == 1:
            print('Invalid version for Transport')
        else:
            self.version = self.sync_handler.version = self.async_handler.version = v

    async def do_pull_rpc_async(self, payload, reply_callback, exiting=False):
        """
        Do an RPC that pulls data from the device
        Parameters
        ----------
        payload: Array of type 'B' with length of RPC data to transmit
        reply_callback: Called after RPC data has been returned
        exiting: Cleanup if a final call during exit

        Returns
        -------
        None
        """
        # Acquire the lock in a worker thread, suspending us while waiting.
        # See https://stackoverflow.com/questions/63420413/how-to-use-threading-lock-in-async-function-while-object-can-be-accessed-from-mu
        await asyncio.get_event_loop().run_in_executor(None, self.lock.acquire)
        success = await self.async_handler.do_rpc(push=False, payload=payload, reply_callback=reply_callback,
                                                  exiting=exiting)
        self.lock.release()

    async def do_push_rpc_async(self, payload, reply_callback, exiting=False):
        """
        Do an RPC that pushes data to the device
        Parameters
        ----------
        payload: Array of type 'B' with length of RPC data to transmit
        reply_callback: Called after RPC data has been returned
        exiting: Cleanup if a final call during exit
        Returns
        -------
        None
        """
        await asyncio.get_event_loop().run_in_executor(None, self.lock.acquire)
        success = await self.async_handler.do_rpc(push=False, payload=payload, reply_callback=reply_callback,
                                                  exiting=exiting)
        self.lock.release()

    def do_pull_rpc_sync(self, payload, reply_callback, exiting=False):
        """
        Do an RPC that pulls data from the device
        Parameters
        ----------
        payload: Array of type 'B' with length of RPC data to transmit
        reply_callback: Called after RPC data has been returned
        exiting: Cleanup if a final call during exit

        Returns
        -------
        None
        """
        with self.lock:
            self.sync_handler.do_rpc(push=False, payload=payload, reply_callback=reply_callback, exiting=exiting)

    def do_push_rpc_sync(self, payload, reply_callback, exiting=False):
        """
        Do an RPC that pushes data to the device
        Parameters
        ----------
        payload: Array of type 'B' with length of RPC data to transmit
        reply_callback: Called after RPC data has been returned
        exiting: Cleanup if a final call during exit

        Returns
        -------
        None
        """
        with self.lock:
            self.sync_handler.do_rpc(push=True, payload=payload, reply_callback=reply_callback, exiting=exiting)


# #####################################

def pack_string_t(s, sidx, x):
    n = len(x)
    return struct.pack_into(str(n) + 's', s, sidx, x)


def unpack_string_t(s, n):
    return (struct.unpack(str(n) + 's', s[:n])[0].strip(b'\x00')).decode('utf-8')


def unpack_int32_t(s):
    return struct.unpack('i', s[:4])[0]


def unpack_uint32_t(s):
    return struct.unpack('I', s[:4])[0]


def unpack_int64_t(s):
    return struct.unpack('q', s[:8])[0]


def unpack_uint64_t(s):
    return struct.unpack('Q', s[:8])[0]


def unpack_int16_t(s):
    return struct.unpack('h', s[:2])[0]


def unpack_uint16_t(s):
    return struct.unpack('H', s[:2])[0]


def unpack_uint8_t(s):
    return struct.unpack('B', s[:1])[0]


def unpack_float_t(s):
    return struct.unpack('f', s[:4])[0]


def unpack_double_t(s):
    return struct.unpack('d', s[:8])[0]


def pack_float_t(s, sidx, x):
    return struct.pack_into('f', s, sidx, x)


def pack_double_t(s, sidx, x):
    return struct.pack_into('d', s, sidx, x)


def pack_int32_t(s, sidx, x):
    return struct.pack_into('i', s, sidx, x)


def pack_uint32_t(s, sidx, x):
    return struct.pack_into('I', s, sidx, x)


def pack_int16_t(s, sidx, x):
    return struct.pack_into('h', s, sidx, x)


def pack_uint16_t(s, sidx, x):
    return struct.pack_into('H', s, sidx, x)


def pack_uint8_t(s, sidx, x):
    return struct.pack_into('B', s, sidx, x)



















