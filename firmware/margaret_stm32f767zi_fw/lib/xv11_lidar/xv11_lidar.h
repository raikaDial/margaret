// xv11_lidar.h
// Ryker Dial
// The purpose of this library is to provide an easy interface for using the
//     XV11 Neato lidar with MBED devices, and to make it easy to interface with
//     multiple lidars at once.

// Parts of this code are inspired by the XV Lidar Controller firmware created by 
//     James LeRoy at getSurreal (copyright 2014-2016)
//         https://github.com/getSurreal/XV_Lidar_Controller
//         http://www.getsurreal.com/products/xv-lidar-controller

// Date Created:    02/09/2017
// Last Modified:   07/26/2017

#ifndef XV11_LIDAR_H
#define XV11_LIDAR_H

// ***** Begin Includes ***** //
#include <mbed.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <PID.h>
// ********** //

extern ros::NodeHandle nh;
extern EventFlags event_flags;

// Number of measurements per LaserScan packet. Must be a multiple of N_DATA_QUADS.
const int NUM_VALS_LASERSCAN = 24; //360;

extern ros::NodeHandle nh;

// ********** Begin defining constants ********** //
const uint32_t XV11_LIDAR_BAUD = 115200;
const uint8_t COMMAND = 0xFA; // Packet header byte
const uint8_t INDEX_LOW = 0xA0; // Lowest index value
const uint8_t INDEX_HI = 0xF9;

const uint8_t N_DATA_QUADS = 4; // Four groups of data elements
const uint8_t N_ELEMENTS_PER_QUAD = 4;  // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB  

// Offsets to bytes within the packet
const uint8_t OFFSET_TO_START = 0;
const uint8_t OFFSET_TO_INDEX = OFFSET_TO_START + 1;
const uint8_t OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
const uint8_t OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
const uint8_t OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
const uint8_t OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + N_DATA_QUADS*N_ELEMENTS_PER_QUAD;
const uint8_t OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
const uint8_t PACKET_LENGTH = OFFSET_TO_CRC_M + 1;

// Offsets to the four elements of each of the four data quads
const uint8_t OFFSET_DATA_DISTANCE_LSB = 0;
const uint8_t OFFSET_DATA_DISTANCE_MSB = OFFSET_DATA_DISTANCE_LSB + 1;
const uint8_t OFFSET_DATA_SIGNAL_LSB = OFFSET_DATA_DISTANCE_MSB + 1;
const uint8_t OFFSET_DATA_SIGNAL_MSB = OFFSET_DATA_SIGNAL_LSB + 1;

enum{INVALID_PACKET, VALID_PACKET};
const uint8_t INVALID_DATA_FLAG = (1 << 7); // Mask for byte 1 of each data quad "Invalid data"

/* REF: https://github.com/Xevel/NXV11/wiki
  The bit 7 of byte 1 seems to indicate that the distance could not be calculated.
  It's interesting to see that when this bit is set, the second byte is always 80, and the values of the first byte seem to be
  only 02, 03, 21, 25, 35 or 50... When it's 21, then the whole block is 21 80 XX XX, but for all the other values it's the
  data block is YY 80 00 00 maybe it's a code to say what type of error ? (35 is preponderant, 21 seems to be when the beam is
  interrupted by the supports of the cover).
*/
const uint8_t STRENGTH_WARNING_FLAG = (1 << 6); // Mask for byte 1 of each data quat "Strength Warning"

/*
  The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance.
  This may happen when the material has a low reflectance (black material...), or when the dot does not have the expected
  size or shape (porous material, transparent fabric, grid, edge of an object...), or maybe when there are parasitic
  reflections (glass... ).
*/
const uint8_t BAD_DATA_MASK = (INVALID_DATA_FLAG | STRENGTH_WARNING_FLAG);

const uint8_t STATE_FIND_COMMAND = 0; // 1st state: find 0xFA (COMMAND) in input stream
const uint8_t STATE_BUILD_PACKET = STATE_FIND_COMMAND + 1; // 2nd state: build the packet
// ******************** //


class Xv11Lidar {
    public:
        // Constructor takes tx and rx pins for communication with xv11 lidar and a pin for pwm control of the lidar's motor
        Xv11Lidar(PinName tx, PinName rx, PinName pwm, uint8_t ID);
        
        void begin();
        void update(); // Compiles LaserScan packets.
        
        // Used for sending the lidar's data to the pc
        std_msgs::UInt16MultiArray m_laserscan_raw_msg;
        int m_laserscan_idx;
        uint8_t m_laserscan_ready;
        uint8_t m_laserscan_start;
        
    private:
        uint8_t m_ID; // ID of this LIDAR. Used to specify frame name.

        RawSerial m_lidar_serial;
        PwmOut m_motor_pwm;
        
        // ***** Control Variables and Containers for Serial RX ISR ***** //
        uint8_t m_state; // The current state of the packet builder. Either it is looking for the next packet or currently building one.
        uint8_t m_packet[PACKET_LENGTH]; // Packet that is currently being built.
        uint8_t m_packet_idx; // Current position in incomplete packet.
        uint8_t m_packet_full[PACKET_LENGTH]; // Stores the most recent complete packet received.
        uint8_t m_packet_ready; // Signals that a new packet is ready for processing.
        uint16_t m_starting_angle; // Angle of the device at the start of the current data collection cycle
        uint32_t m_num_scans_rxd;
        uint16_t m_ranges[NUM_VALS_LASERSCAN];
        uint16_t m_intensities[NUM_VALS_LASERSCAN];
        // ********** //
        
        // ***** PID Controller Configuration ***** //
        double m_Kp, m_Ti, m_Td; // PID tuning values for motor pwm.
        float m_pid_loop_rate; // How often the PID controller updates.
        double m_motor_rpm_goal; // Ideal value of the motor's rpm.
        float m_motor_rpm_min, m_motor_rpm_max; // Limits on motorrpm.
        float m_motor_pwm_min, m_motor_pwm_max; // Limits on motor pwm duty cycle.
        PID m_motor_controller;
        // *********** //
        
        double m_motor_rpm;
        float m_motor_rpm_sum; // Cumulative sum of motor rpm, for calculating average for estimating motion of points.
        int m_num_good_readings;

        void rxISR(void); // Builds lidar packets

        // Helper functions for processing lidar packets.
        uint8_t validatePacket();
        void processData(uint16_t num_quad, uint16_t & distance, uint16_t & intensity);
        void publishLaserScan();
        void storeLaserscanPacketHeader();
        
}; // end class Xv11Lidar

#endif // XV11_LIDAR_H