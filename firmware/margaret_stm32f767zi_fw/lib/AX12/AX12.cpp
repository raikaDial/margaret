/* mbed AX-12+ Servo Library
 *
 * Copyright (c) 2010, cstyles (http://mbed.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "AX12.h"
#include "mbed.h"

AX12::AX12(PinName tx, PinName rx, PinName tx_en, int ID, int baud)
        : _ax12(tx, rx), _ID(ID), _baud(baud), _tx_en(tx_en),  _cw_limit_baseline(105),
          _ccw_limit_baseline(195), _cw_limit(_cw_limit_baseline), _ccw_limit(_ccw_limit_baseline), 
          _rotation_speed_baseline(0.1), _rotation_speed(_rotation_speed_baseline), _rotation_direction(AX12_ROT_CW), 
          _cw_limit_changed(true), _ccw_limit_changed(true), _max_offset_rotation_speed(0.044), _add_offset(0)
{
    _ax12.baud(_baud);
        
    // Start up publisher to signal that lidar has reached the end of its pan
    char* topic = new char[32];
    sprintf(topic, "ax12_done_panning_%d", _ID);
    _pan_finished_pub = new ros::Publisher(topic, &_pan_finished_msg);
    nh.advertise(*_pan_finished_pub);
    
    // Move to the starting position
    SetCRSpeed(1);
    SetGoal(_cw_limit);
}


// Set the mode of the servo
//  0 = Positional (0-300 degrees)
//  1 = Rotational -1 to 1 speed
int AX12::SetMode(int mode) {

    if (mode == 1) { // set CR
        SetCWLimit(0);
        SetCCWLimit(0);
        SetCRSpeed(0.0);
    } else {
        SetCWLimit(0);
        SetCCWLimit(300);
        SetCRSpeed(0.0);
    }
    return(0);
}


// if flag[0] is set, were blocking
// if flag[1] is set, we're registering
// they are mutually exclusive operations
int AX12::SetGoal(int degrees, int flags) {
    // Make sure the angle is valid
    if(degrees < 0 || degrees > 300)
        return -1;

    char reg_flag = 0;
    char data[2];

    // set the flag is only the register bit is set in the flag
    if (flags == 0x2) {
        reg_flag = 1;
    }

    // 1023 / 300 * degrees
    short goal = (1023 * degrees) / 300;
#ifdef AX12_DEBUG
    pc.printf("SetGoal to 0x%x\n",goal);
#endif

    data[0] = goal & 0xff; // bottom 8 bits
    data[1] = goal >> 8;   // top 8 bits

    // write the packet, return the error code
    int rVal = write(_ID, AX12_REG_GOAL_POSITION, 2, data, reg_flag);

    if (flags == 1) {
        // block until it comes to a halt
        while (isMoving()) {}
    }
    return(rVal);
}


// Set continuous rotation speed from -1 to 1
int AX12::SetCRSpeed(float speed) {

    // bit 10     = direction, 0 = CCW, 1=CW
    // bits 9-0   = Speed
    char data[2];

    int goal = (0x3ff * abs(speed));

    // Set direction CW if we have a negative speed
    if (speed < 0) {
        goal |= (0x1 << 10);
    }

    data[0] = goal & 0xff; // bottom 8 bits
    data[1] = goal >> 8;   // top 8 bits

    // write the packet, return the error code
    int rVal = write(_ID, 0x20, 2, data);

    return(rVal);
}

int AX12::SetCWLimit (int degrees) {

    char data[2];

    // 1023 / 300 * degrees
    short limit = (1023 * degrees) / 300;

#ifdef AX12_DEBUG
    pc.printf("SetCWLimit to 0x%x\n",limit);
#endif

    data[0] = limit & 0xff; // bottom 8 bits
    data[1] = limit >> 8;   // top 8 bits

    // write the packet, return the error code
    return (write(_ID, AX12_REG_CW_LIMIT, 2, data));

}

int AX12::SetCCWLimit (int degrees) {

    char data[2];

    // 1023 / 300 * degrees
    short limit = (1023 * degrees) / 300;

#ifdef AX12_DEBUG
    pc.printf("SetCCWLimit to 0x%x\n",limit);
#endif

    data[0] = limit & 0xff; // bottom 8 bits
    data[1] = limit >> 8;   // top 8 bits

    // write the packet, return the error code
    return (write(_ID, AX12_REG_CCW_LIMIT, 2, data));
}


int AX12::SetID (int CurrentID, int NewID) {

    char data[1];
    data[0] = NewID;

#ifdef AX12_DEBUG
    pc.printf("Setting ID from 0x%x to 0x%x\n",CurrentID,NewID);
#endif

    return (write(CurrentID, AX12_REG_ID, 1, data));

}


int AX12::SetBaud (int baud) {

    char data[1];
    data[0] = baud;

#ifdef AX12_DEBUG
    pc.printf("Setting Baud rate to %d\n",baud);
#endif

    return (write(0xFE, AX12_REG_BAUD, 1, data));

}

int AX12::SetReturnDelay(uint8_t delay) {
    
    char data[1];
    data[0] = delay;
    
#ifdef AX12_DEBUG
    pc.printf("Setting return delay to %d\n",delay);
#endif

    return (write(0xFE, AX12_REG_RETURN_DELAY, 1, data));

}

// return 1 is the servo is still in flight
int AX12::isMoving(void) {

    char data[1];
    read(_ID,AX12_REG_MOVING,1,data);
    return(data[0]);
}


void AX12::trigger(void) {

    char TxBuf[16];
    char sum = 0;

#ifdef AX12_TRIGGER_DEBUG
    // Build the TxPacket first in RAM, then we'll send in one go
    pc.printf("\nTriggered\n");
    pc.printf("\nTrigger Packet\n  Header : 0xFF, 0xFF\n");
#endif

    TxBuf[0] = 0xFF;
    TxBuf[1] = 0xFF;

    // ID - Broadcast
    TxBuf[2] = 0xFE;
    sum += TxBuf[2];

#ifdef AX12_TRIGGER_DEBUG
    pc.printf("  ID : %d\n",TxBuf[2]);
#endif

    // Length
    TxBuf[3] = 0x02;
    sum += TxBuf[3];

#ifdef AX12_TRIGGER_DEBUG
    pc.printf("  Length %d\n",TxBuf[3]);
#endif

    // Instruction - ACTION
    TxBuf[4] = 0x04;
    sum += TxBuf[4];

#ifdef AX12_TRIGGER_DEBUG
    pc.printf("  Instruction 0x%X\n",TxBuf[5]);
#endif

    // Checksum
    TxBuf[5] = 0xFF - sum;
#ifdef AX12_TRIGGER_DEBUG
    pc.printf("  Checksum 0x%X\n",TxBuf[5]);
#endif

    ax12_bus_mutex.lock();
    _tx_en = 1;
    // Transmit the packet in one burst with no pausing
    for (int i = 0; i < 6 ; i++) {
        _ax12.putc(TxBuf[i]);
    }
    wait_us(20);
    _tx_en = 0;
    ax12_bus_mutex.unlock();
    // This is a broadcast packet, so there will be no reply
    return;
}


float AX12::GetPosition(void) {

#ifdef AX12_DEBUG
    pc.printf("\nGetPosition(%d)",_ID);
#endif

    char data[2];

    int ErrorCode = read(_ID, AX12_REG_POSITION, 2, data);
    
    if(ErrorCode == -1) // If read was unsuccessful, return the most recent goal.
        return -1;
        
    short position = data[0] + (data[1] << 8);
    float angle = (position * 300)/1024.0;

    return (angle);
}


float AX12::GetTemp (void) {

#ifdef AX12_DEBUG
    pc.printf("\nGetTemp(%d)",_ID);
#endif

    char data[1];
    int ErrorCode = read(_ID, AX12_REG_TEMP, 1, data);
    float temp = data[0];
    return(temp);
}


float AX12::GetVolts (void) {

#ifdef AX12_DEBUG
    pc.printf("\nGetVolts(%d)",_ID);
#endif

    char data[1];
    int ErrorCode = read(_ID, AX12_REG_VOLTS, 1, data);
    float volts = data[0]/10.0;
    return(volts);
}


int AX12::read(int ID, int start, int bytes, char* data) {

    char PacketLength = 0x4;
    char TxBuf[16];
    char sum = 0;
    uint8_t Status[16];
    for(int i=0; i < 16; ++i)
        Status[i] = 0;

    Status[4] = 0xFE; // return code

#ifdef AX12_READ_DEBUG
    pc.printf("\nread(%d,0x%x,%d,data)\n",ID,start,bytes);
#endif

    // Build the TxPacket first in RAM, then we'll send in one go
#ifdef AX12_READ_DEBUG
    pc.printf("\nInstruction Packet\n  Header : 0xFF, 0xFF\n");
#endif

    TxBuf[0] = 0xff;
    TxBuf[1] = 0xff;

    // ID
    TxBuf[2] = ID;
    sum += TxBuf[2];

#ifdef AX12_READ_DEBUG
    pc.printf("  ID : %d\n",TxBuf[2]);
#endif

    // Packet Length
    TxBuf[3] = PacketLength;    // Length = 4 ; 2 + 1 (start) = 1 (bytes)
    sum += TxBuf[3];            // Accululate the packet sum

#ifdef AX12_READ_DEBUG
    pc.printf("  Length : 0x%x\n",TxBuf[3]);
#endif

    // Instruction - Read
    TxBuf[4] = 0x2;
    sum += TxBuf[4];

#ifdef AX12_READ_DEBUG
    pc.printf("  Instruction : 0x%x\n",TxBuf[4]);
#endif

    // Start Address
    TxBuf[5] = start;
    sum += TxBuf[5];

#ifdef AX12_READ_DEBUG
    pc.printf("  Start Address : 0x%x\n",TxBuf[5]);
#endif

    // Bytes to read
    TxBuf[6] = bytes;
    sum += TxBuf[6];

#ifdef AX12_READ_DEBUG
    pc.printf("  No bytes : 0x%x\n",TxBuf[6]);
#endif

    // Checksum
    TxBuf[7] = 0xFF - sum;
#ifdef AX12_READ_DEBUG
    pc.printf("  Checksum : 0x%x\n",TxBuf[7]);
#endif
    
    // Transmit the packet in one burst with no pausing
    ax12_bus_mutex.lock(); // acquire the serial bus
    
    // Clear the rx buffer of any orphaned bytes
    while(_ax12.readable()) 
        _ax12.getc();
    
    _tx_en = 1;
    for (int i = 0; i<8 ; i++) {
        _ax12.putc(TxBuf[i]);
    }
   
    // Wait for the bytes to be transmitted
    wait_us(20);
    _tx_en = 0;
        
    // Skip if the read was to the broadcast address
    if (_ID != 0xFE) {
        // response packet is always 6 + bytes
        // 0xFF, 0xFF, ID, Length, Error, Param(s), Checksum
        // timeout is a little more than the time to transmit
        // the packet back, i.e. (6+bytes)*10 bit periods
        int plen = 0;
        Timer timeout;
        timeout.reset();
        timeout.start();
        while ((timeout.read_us() < ((6+bytes)*10*4)) && (plen<(6+bytes))) {
            if (_ax12.readable()) {
                Status[plen++] = _ax12.getc();
                timeout.reset();
            }
        }
        timeout.stop();
        ax12_bus_mutex.unlock();
        
        if (timeout.read_us() >= ((6+bytes)*10*4)) { // If we timed out
          return(-1);
        }

        // Exit in error if the length would make us go out of bounds for Status.
        if(Status[3] > 16 - 6) {
             return -1;
        }
        
        // Verify the checksum
        char sum = 0;
        for(int i=0; i < Status[3]+1; ++i) {
             sum += Status[2+i];
        }
        if( 0xFF - sum != Status[4 + Status[3] - 1] ) {
            return -1;
        }
             
        // Copy the data from Status into data for return
        for (int i=0; i < Status[3]-2 ; i++) {
            data[i] = Status[5+i];
        }

#ifdef AX12_READ_DEBUG
        pc.printf("\nStatus Packet\n");
        pc.printf("  Header : 0x%x\n",Status[0]);
        pc.printf("  Header : 0x%x\n",Status[1]);
        pc.printf("  ID : 0x%x\n",Status[2]);
        pc.printf("  Length : 0x%x\n",Status[3]);
        pc.printf("  Error Code : 0x%x\n",Status[4]);

        for (int i=0; i < Status[3]-2; i++) {
            pc.printf("  Data : 0x%x\n",Status[5+i]);
        }

        pc.printf("  Checksum : 0x%x\n",Status[5+(Status[3]-2)]);
#endif

    } // if (ID!=0xFE)
    
    return(Status[4]);
}


int AX12::write(int ID, int start, int bytes, char* data, int flag) {
// 0xff, 0xff, ID, Length, Intruction(write), Address, Param(s), Checksum

    char TxBuf[16];
    char sum = 0;
    char Status[6];
    for(int i=0; i<6; ++i)
        Status[i] = 0;

#ifdef AX12_WRITE_DEBUG
    pc.printf("\nwrite(%d,0x%x,%d,data,%d)\n",ID,start,bytes,flag);
#endif

    // Build the TxPacket first in RAM, then we'll send in one go
#ifdef AX12_WRITE_DEBUG
    pc.printf("\nInstruction Packet\n  Header : 0xFF, 0xFF\n");
#endif

    TxBuf[0] = 0xff;
    TxBuf[1] = 0xff;

    // ID
    TxBuf[2] = ID;
    sum += TxBuf[2];

#ifdef AX12_WRITE_DEBUG
    pc.printf("  ID : %d\n",TxBuf[2]);
#endif

    // packet Length
    TxBuf[3] = 3+bytes;
    sum += TxBuf[3];

#ifdef AX12_WRITE_DEBUG
    pc.printf("  Length : %d\n",TxBuf[3]);
#endif

    // Instruction
    if (flag == 1) {
        TxBuf[4]=0x04;
        sum += TxBuf[4];
    } else {
        TxBuf[4]=0x03;
        sum += TxBuf[4];
    }

#ifdef AX12_WRITE_DEBUG
    pc.printf("  Instruction : 0x%x\n",TxBuf[4]);
#endif

    // Start Address
    TxBuf[5] = start;
    sum += TxBuf[5];

#ifdef AX12_WRITE_DEBUG
    pc.printf("  Start : 0x%x\n",TxBuf[5]);
#endif

    // data
    for (char i=0; i<bytes ; i++) {
        TxBuf[6+i] = data[i];
        sum += TxBuf[6+i];

#ifdef AX12_WRITE_DEBUG
        pc.printf("  Data : 0x%x\n",TxBuf[6+i]);
#endif

    }

    // checksum
    TxBuf[6+bytes] = 0xFF - sum;

#ifdef AX12_WRITE_DEBUG
    pc.printf("  Checksum : 0x%x\n",TxBuf[6+bytes]);
#endif
    
    ax12_bus_mutex.lock();
    
    while(_ax12.readable()) 
        _ax12.getc();
    
    // Transmit the packet in one burst with no pausing
    _tx_en = 1;
    for (int i = 0; i < (7 + bytes); i++) {
        _ax12.putc(TxBuf[i]);
    }

    // Wait for data to transmit
    wait_us(20);
    _tx_en = 0;
    
    // make sure we have a valid return
    Status[4]=0x00;
    
    // we'll only get a reply if it was not broadcast
    if (_ID!=0xFE) {
        // response packet is always 6 bytes
        // 0xFF, 0xFF, ID, Length Error, Param(s) Checksum
        // timeout is a little more than the time to transmit
        // the packet back, i.e. 60 bit periods, round up to 100
        //int timeout = 0;
        
        int plen = 0;
        Timer timeout;
        timeout.reset();
        timeout.start();

        while (timeout.read_us() < 100*3) {
//            if(_ax12.readable()) {
//                if(plen < 6) {
//                    Status[plen++] = _ax12.getc();
//                    plen++;
//                    //timeout.reset();
//                }
//                else {
//                    break;
//                }
//            }
        }
        timeout.stop();
        ax12_bus_mutex.unlock(); // release the dynamixel serial bus
        
        if(timeout.read_us() > 300)
            return -1;
        
        // Build the TxPacket first in RAM, then we'll send in one go
#ifdef AX12_WRITE_DEBUG
        pc.printf("\nStatus Packet\n  Header : 0x%X, 0x%X\n",Status[0],Status[1]);
        pc.printf("  ID : %d\n",Status[2]);
        pc.printf("  Length : %d\n",Status[3]);
        pc.printf("  Error : 0x%x\n",Status[4]);
        pc.printf("  Checksum : 0x%x\n",Status[5]);
#endif

    }
    
    return(Status[4]); // return error code
}


void AX12::StartPanning(void) {
}


void AX12::panThreadFunc(void) {
    #ifdef STOCHASTIC_DITHERING_NONADAPTIVE
        if(_add_offset == 0) {
            UpdateStochasticDithering();
        }
        _add_offset = (++_add_offset)%10;
    #endif
    if(!isMoving()) {
        if(_rotation_direction == AX12_ROT_CCW) {
            // If the panning limit changed, update here.
            if(_cw_limit_changed) {
                SetCWLimit(_cw_limit);
                _cw_limit_changed = false;
            }
            
            // Start movement to the next limit
            _rotation_direction = AX12_ROT_CW;
            SetCRSpeed(_rotation_speed);
            SetGoal(_cw_limit);
        }
        else if(_rotation_direction == AX12_ROT_CW) {
            if(_ccw_limit_changed) {
                SetCCWLimit(_ccw_limit);
                _ccw_limit_changed = false;
            }
            
            _rotation_direction = AX12_ROT_CCW;
            //SetCRSpeed(_rotation_speed);
            //UpdateStochasticDithering();
            SetCRSpeed(0.25f);                 
            SetGoal(_ccw_limit);
            _pan_finished_pub -> publish(&_pan_finished_msg);
            
        }
    }
    if(_rotation_speed_changed) {
        if(_rotation_direction == AX12_ROT_CW) {
             SetCRSpeed(_rotation_speed);
             _rotation_speed_changed = false;
        }
    }
         
    if(_ID == 1)
        led3 = !led3;
}


void AX12::SetPanningParams(int cw_limit, int ccw_limit, int rotation_speed, bool set_baselines) {
    // Set CW and CCW limits, making sure they are not out of bounds and are proper intervals.
    if( (cw_limit < ccw_limit) && (cw_limit >= 0) && (ccw_limit <= 300) ) {
        _cw_limit = cw_limit;
        _ccw_limit = ccw_limit;
        
        // Flag that limits have changed. These are used to prevent the servo from getting
        //     stuck outside the panning limits.
        _cw_limit_changed = true;
        _ccw_limit_changed = true;
        
        // Adjust the baselines if requested
        if(set_baselines) {
            _cw_limit_baseline = _cw_limit;
            _ccw_limit_baseline =  _ccw_limit;
        }  
    }
    
    // Set rotation step time
    float rotation_speed_f = rotation_speed/1023.0f;
    if(rotation_speed_f >= 0.0f && rotation_speed_f <= 1.0f) {

        // Store new rotation speed. The servo's actual speed will be updated
        //     the next time it pans clockwise, or in the next update step
        //     if it is already moving clockwise.
        _rotation_speed = rotation_speed_f;
        _rotation_speed_changed = true;

        if(set_baselines) {
            _rotation_speed_baseline = _rotation_speed;
        }        
    }
}

void AX12::UpdateStochasticDithering() {
    // Get a random byte for the offset in rotation step
    uint8_t random_bytes[1];
    size_t output_length;
    
    if(trng_get_bytes(&trng_obj, random_bytes, 1, &output_length) == 0) { // Returns 0 on success
        //random_bytes[0] > 127 ? add_offset = true : add_offset = false;
        // Generate new rotation speed
        // int rotation_speed_new = (_rotation_speed_baseline + _max_offset_rotation_speed*add_offset)*1023;
        int rotation_speed_new = (_rotation_speed_baseline + (random_bytes[0]/255.0)*2*_max_offset_rotation_speed - _max_offset_rotation_speed)*1023;
        
        // Set new panning params
        SetPanningParams(-1, -1, rotation_speed_new, false);    
    }
//    int rotation_speed_new = (_rotation_speed_baseline + _add_offset*_max_offset_rotation_speed)*1023;
//    _add_offset = !_add_offset;
//
//    // Set new panning params
//    SetPanningParams(-1, -1, rotation_speed_new, false);
    
}
//    // Get three random bytes:
//    //  One for the offset in CW limit
//    //  One for the offset in CCW limit
//    //  One for the offset in rotation step
//    uint8_t random_bytes[3];
//    size_t output_length;
//    
//    if(!trng_get_bytes(&trng_obj, random_bytes, 3, &output_length)) { // Returns 0 on success
//        // Generate new panning params
//        int cw_limit_new = _cw_limit_baseline + ((int)random_bytes[0])%(2*max_offset_panning_limits) - max_offset_panning_limits;
//        int ccw_limit_new = _ccw_limit_baseline + ((int)random_bytes[1])%(2*max_offset_panning_limits) - max_offset_panning_limits;
//        int rotation_speed_new = (_rotation_speed_baseline + (random_bytes[2]/255.0)*2*max_offset_rotation_speed - max_offset_rotation_speed)*1023;
//        
//        // Set new panning params
//        SetPanningParams(cw_limit_new, ccw_limit_new, rotation_speed_new, false);
//    }