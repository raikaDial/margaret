/* mbed AX-12+ Servo Library
 *
 * Copyright (c) 2010, cstyles (http://mbed.org)
 * Modified 2017 by Ryker Dial
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
 
 /* Warning: This library uses a global mutex, ax12_bus_mutex, to arbitrate
  * control of the serial bus. Mutexes do not work in interrupts, so using class 
  * functions within interrupts may result in collisions on the serial bus.
  */

#ifndef MBED_AX12_H
#define MBED_AX12_H

#include "mbed.h"
#include <ros.h>
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Empty.h"
#include "trng_api.h" // Access to TRNG Hardware for Stochastic Dithering

// ***** Options ***** //
#define STOCHASTIC_DITHERING_NONADAPTIVE /* Dither lidar speeds using hardware random number generator */
// ********** //

extern ros::NodeHandle nh;

// Pan param verification message and publisher
//extern std_msgs::Int16MultiArray configured_pan_param_msg;
//extern ros::Publisher configured_pan_param_pub;

// User LEDs for Debugging
extern DigitalOut led1;
extern DigitalOut led2;
extern DigitalOut led3;

extern Serial pc;

extern trng_t trng_obj;

//#define AX12_WRITE_DEBUG 0
//#define AX12_READ_DEBUG 0
//#define AX12_TRIGGER_DEBUG 0
//#define AX12_DEBUG 0

#define AX12_REG_ID 0x3
#define AX12_REG_BAUD 0x4
#define AX12_REG_RETURN_DELAY 0x05
#define AX12_REG_CW_LIMIT 0x06
#define AX12_REG_CCW_LIMIT 0x08
#define AX12_REG_GOAL_POSITION 0x1E
#define AX12_REG_MOVING_SPEED 0x20
#define AX12_REG_VOLTS 0x2A
#define AX12_REG_TEMP 0x2B
#define AX12_REG_MOVING 0x2E
#define AX12_REG_POSITION 0x24

#define AX12_MODE_POSITION  0
#define AX12_MODE_ROTATION  1

#define AX12_ROT_CW 0
#define AX12_ROT_CCW 1

/** Servo control class, based on a PwmOut
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "AX12.h"
 * 
 * int main() {
 * 
 *   AX12 myax12 (p9, p10, p11, 1);
 *
 *   while (1) {
 *       myax12.SetGoal(0);    // go to 0 degrees
 *       wait (2.0);
 *       myax12.SetGoal(300);  // go to 300 degrees
 *       wait (2.0);
 *   }
 * }
 * @endcode
 */
class AX12 {

public:

    /** Create an AX12 servo object connected to the specified serial port, with the specified ID
     *
     * @param pin tx pin
     * @param pin rx pin 
     * @param pin tx_en pin, direction pin for serial bus
     * @param int ID, the Bus ID of the servo 1-255 
     */
    AX12(PinName TX, PinName RX, PinName tx_en, int ID, int baud=1000000);

    /** Set the mode of the servo
     * @param mode
     *    0 = Positional, default
     *    1 = Continuous rotation
     */
    int SetMode(int mode);

    /** Set baud rate of all attached servos
     * @param mode
     *    0x01 = 1,000,000 bps
     *    0x03 =   500,000 bps
     *    0x04 =   400,000 bps
     *    0x07 =   250,000 bps
     *    0x09 =   200,000 bps
     *    0x10 =   115,200 bps
     *    0x22 =    57,600 bps
     *    0x67 =    19,200 bps
     *    0xCF =     9,600 bp
     */
    int SetBaud(int baud);

    /**
    * @param delay 0-255. Actual delay is 2us times this number.
    */
    int SetReturnDelay(uint8_t delay);

    /** Set goal angle in integer degrees, in positional mode
     *
     * @param degrees 0-300
     * @param flags, defaults to 0
     *    flags[0] = blocking, return when goal position reached 
     *    flags[1] = register, activate with a broadcast trigger
     *
     */
    int SetGoal(int degrees, int flags = 0);


    /** Set the speed of the servo in continuous rotation mode
     *
     * @param speed, -1.0 to 1.0
     *   -1.0 = full speed counter clock wise
     *    1.0 = full speed clock wise
     */
    int SetCRSpeed(float speed);

    /** Set the clockwise limit of the servo
     *
     * @param degrees, 0-300
     */
    int SetCWLimit(int degrees);
    
    /** Set the counter-clockwise limit of the servo
     *
     * @param degrees, 0-300
     */
    int SetCCWLimit(int degrees);

    // Change the ID

    /** Change the ID of a servo
     *
     * @param CurentID 1-255
     * @param NewID 1-255
     *
     * If a servo ID is not known, the broadcast address of 0 can be used for CurrentID.
     * In this situation, only one servo should be connected to the bus
     */
    int SetID(int CurrentID, int NewID);


    /** Poll to see if the servo is moving
     *
     * @returns true is the servo is moving
     */
    int isMoving(void);

    /** Send the broadcast "trigger" command, to activate any outstanding registered commands
     */
    void trigger(void);

    /** Read the current angle of the servo
     *
     * @returns float in the range 0.0-300.0.
     */
    float GetPosition();

    /** Read the temperature of the servo
     *
     * @returns float temperature 
     */
    float GetTemp(void);

    /** Read the supply voltage of the servo
     *
     * @returns float voltage
     */
    float GetVolts(void);
    
    /** Start the panning thread
     */
    void StartPanning(void);
    
    /** Change the panning parameters
     *
     * @ param
     * @ param
     * @ param
     */
    void SetPanningParams(int cw_limit, int ccw_limit, int rotation_step = -1, bool set_baselines = true);
    
//    void PublishPanningParams();
    
    /** Update Stochastic Dithering
     */
    void UpdateStochasticDithering();

    int read(int ID, int start, int length, char* data);
    int write(int ID, int start, int length, char* data, int flag=0);
    
    void panThreadFunc(void);

private:
    // Configuration Variables
    RawSerial _ax12;
    int _ID;
    int _baud;
    DigitalOut _tx_en;
    
    // Variables for Panning
    int _cw_limit_baseline, _ccw_limit_baseline; // CW and CCW panning limits before stochastic dithering
    int _cw_limit, _ccw_limit; // CW and CCW panning limits after stochastic dithering
    float _rotation_speed_baseline; // Servo speed scaling value before stochastic dithering
    float _rotation_speed; // Servo speed scaling value after stochastic dithering
    uint8_t _rotation_direction; // Current rotation direction, either CW (0) or CCW (1)
    bool _cw_limit_changed, _ccw_limit_changed; // Signals that panning limits have changed.
    bool _rotation_speed_changed;
    
    uint8_t _add_offset;
    
    // Dithering
    float _max_offset_rotation_speed;
    
    std_msgs::Empty _pan_finished_msg;
    ros::Publisher* _pan_finished_pub;

    Thread _pan_thread;
};

static Mutex ax12_bus_mutex; // Only one servo can use the serial bus at a time.

#endif
