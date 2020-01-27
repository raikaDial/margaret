// xv11_lidar.cpp

// Date Created:    February 9, 2017
// Last Modified:   July 21, 2017

// ***** Begin Includes ***** //
#include "xv11_lidar.h"
// ********** //

extern Serial pc;


Xv11Lidar::Xv11Lidar(PinName tx, PinName rx, PinName pwm, int ID)
    : m_ID(ID), m_lidar_serial(tx, rx, XV11_LIDAR_BAUD), m_motor_pwm(pwm), m_state(STATE_FIND_COMMAND), m_packet_idx(0), m_packet_ready(0), m_num_scans_rxd(0),
      m_Kp(1.0f), m_Ti(0.5f), m_Td(0.0f), m_pid_loop_rate(0.002f), m_motor_rpm_goal(310.0f), m_motor_rpm_min(305.0f), m_motor_rpm_max(315.0f),
      m_motor_pwm_min(0.4f), m_motor_pwm_max(1.0f), m_motor_controller(m_Kp, m_Ti, m_Td, m_pid_loop_rate), m_motor_rpm_sum(0.0f), m_num_good_readings(0)
{   
    // Setup buffers for range and intensity data.
    for(int i=0; i<NUM_VALS_LASERSCAN; ++i)
    {
        m_ranges[i] = 0;
        m_intensities[i] = 0;
    }

    m_laserscan_idx = 0;
    m_laserscan_ready = 0;
    m_laserscan_start = 1;

    // Configure static values in LaserScan message.
    char buff[32];
    sprintf(buff, "laser_link_%d", m_ID);
    m_laserscan_msg.header.frame_id = "laser_link"; //buff;
    m_laserscan_msg.angle_increment = 3.14159265359/180.0;
    m_laserscan_msg.range_min = 0.02;
    m_laserscan_msg.range_max = 6.0;
    m_laserscan_msg.ranges_length = NUM_VALS_LASERSCAN;
    m_laserscan_msg.intensities_length = NUM_VALS_LASERSCAN;
    m_laserscan_msg.ranges = new float[NUM_VALS_LASERSCAN];
    m_laserscan_msg.intensities = new float[NUM_VALS_LASERSCAN];

    // Start looking for the start of the packet
    m_lidar_serial.attach(this, &Xv11Lidar::rxISR, Serial::RxIrq);
}

void Xv11Lidar::begin()
{
    // Initialize ROS publisher
    // char buff[32];
    // sprintf(buff, "scan_%d", m_ID);
    // m_laserscan_pub = new ros::Publisher(buff, &m_laserscan_msg);
    // nh.advertise(*m_laserscan_pub);

    // Initialize Motor PWM
    m_motor_pwm.period(1/32768.0f); // 32.768 kHz pwm frequency
    m_motor_pwm.write(0.70f); // start with 70% duty cycle
        
    // ***** Initialize PID control of motor ***** //
    m_motor_controller.setInputLimits(m_motor_rpm_min, m_motor_rpm_max);
    m_motor_controller.setOutputLimits(m_motor_pwm_min, m_motor_pwm_max);
    m_motor_controller.setMode(AUTO_MODE);
    m_motor_controller.setSetPoint(m_motor_rpm_goal);
    // ********** //
}

void Xv11Lidar::update() {
    if(m_packet_ready)
    {
        m_packet_ready = 0;
        // Process the packet
        if(validatePacket())
        {
            // Get current device angle
            int current_angle = (m_packet_full[OFFSET_TO_INDEX] - INDEX_LOW)*N_DATA_QUADS;
        
            // Store starting angle and timestamp if start of new data collection cycle
            if(m_laserscan_start) 
            {
                //pc.printf("New scan");
                // Store starting angle and starting timestamp
                m_laserscan_msg.header.stamp = nh.now();
                m_laserscan_msg.angle_min = (float) current_angle*M_PI/180.0;
                m_laserscan_msg.angle_max = (float) ((current_angle + NUM_VALS_LASERSCAN-1)%360)*M_PI/180.0;
                m_starting_angle = current_angle;
                m_laserscan_start = 0;
            }

            // Calculate the speed of the motor. Value reported by Neato LIDAR is RPM*64.
            m_motor_rpm = ((m_packet_full[OFFSET_TO_SPEED_MSB] << 8) | m_packet_full[OFFSET_TO_SPEED_LSB])/64.0;
            
            // Store values for obtaining average motor rpm
            m_motor_rpm_sum += m_motor_rpm;
            ++m_num_good_readings;
            
            // Update the PID controller
            m_motor_controller.setProcessValue(m_motor_rpm);
            m_motor_pwm.write(m_motor_controller.compute());

            // Determine the index of the current data
            //pc.printf("Curr: %d start: %d\n", current_angle, m_starting_angle);
            int idx = current_angle - m_starting_angle;
            while(idx < 0) idx += 360;

            if(idx >= NUM_VALS_LASERSCAN) // We've missed the ending packet for this cycle and overshot;
            {
                while(m_laserscan_idx < NUM_VALS_LASERSCAN)
                {
                    m_ranges[m_laserscan_idx] = 0;
                    m_intensities[m_laserscan_idx] = 0;
                    ++m_laserscan_idx;
                }

                //pc.printf("Overshot packet\n");
                publishLaserScan();

                // Create new laserscan with current packet.
                m_laserscan_msg.header.stamp = nh.now();
                m_laserscan_msg.angle_min = (float) current_angle*M_PI/180.0;
                m_laserscan_msg.angle_max = (float) ((current_angle + NUM_VALS_LASERSCAN-1)%360)*M_PI/180.0;
                m_starting_angle = current_angle;
                m_laserscan_start = 0;
            }
            else
            {
                //pc.printf("idx: %d\n", idx);
                while((idx - m_laserscan_idx) > 1 ) // If we dropped a packet fill previous values with 0.
                {
                    m_ranges[m_laserscan_idx] = 0;
                    m_intensities[m_laserscan_idx] = 0;
                    ++m_laserscan_idx;
                    //pc.printf("Dropped idx: %d\n", m_laserscan_idx);                     
                }

                // Now process the current packet.
                for(int i=0; i<N_DATA_QUADS; ++i)
                {
                    uint16_t distance, intensity;
                    processData(i, distance, intensity);
                    m_ranges[m_laserscan_idx] = distance/1000.0;
                    m_intensities[m_laserscan_idx] = intensity;
                    
                    ++m_laserscan_idx;
                    //pc.printf("Packet idx: %d\n", m_laserscan_idx);   

                    if(m_laserscan_idx == (NUM_VALS_LASERSCAN-1))
                    {
                        //pc.printf("Publish laserscan\n");
                        publishLaserScan();
                    }
                }
            }
        }
    }
}

// Determines whether a packet is valid or not with cyclic redundancy checking. 
uint8_t Xv11Lidar::validatePacket() {
    uint32_t chk32 = 0;
    const uint16_t BYTES_TO_CHECK = PACKET_LENGTH - 2;
    const uint16_t BYTES_CRC = BYTES_TO_CHECK/2;
    uint16_t calc_CRC[BYTES_CRC];

    // Initialize array elements to zero.
    for(int i=0; i < BYTES_CRC; ++i) {
        calc_CRC[i] = 0;
    }

    // Perform the cyclic redundancy check
    for(int i=0; i < BYTES_TO_CHECK; i+=2) {
        calc_CRC[i/2] = m_packet_full[i] + (m_packet_full[i+1] << 8);
    }
    for(int i=0; i < BYTES_CRC; ++i) {
        chk32 = (chk32 << 1) + calc_CRC[i];
    }
    uint32_t checksum = ((chk32 & 0x7FFF) + (chk32 >> 15)) & 0x7FFF;

    uint8_t CRC_L = checksum & 0xFF;
    uint8_t CRC_M = checksum >> 8;

    if((CRC_L == m_packet_full[OFFSET_TO_CRC_L]) && (CRC_M == m_packet_full[OFFSET_TO_CRC_M])) {        
        return VALID_PACKET;
    }
    else {
        return INVALID_PACKET;
    }
}

// Given an index to a specific pair of measurements in the data quad and pointers
//     to store the measurements, extracts the distance in millimeters and the 
//     the signal intensity. The distance is set to zero if the INVALID_DATA_FLAG
//     or STRENGTH_WARNING_FLAG is set.
/*
Data 0 to Data 3 are the 4 readings. Each one is 4 bytes long, and organized as follows :
    byte 0 : <distance 7:0>
    byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
    byte 2 : <signal strength 7:0>
    byte 3 : <signal strength 15:8>
*/
void Xv11Lidar::processData(uint16_t num_quad, uint16_t & distance, uint16_t & intensity) {   
    // Process the distance datum
    uint8_t offset_distance = OFFSET_TO_4_DATA_READINGS + num_quad*N_DATA_QUADS + OFFSET_DATA_DISTANCE_LSB;
    uint8_t distance_MSB = m_packet_full[offset_distance + 1];

    if(distance_MSB & BAD_DATA_MASK) { // Set distance to 0 if any bad data flags are set.
        distance = 0;
        intensity = 0;
    }
    else { // Extract the distance in mm to the data point
        uint8_t distance_LSB = m_packet_full[offset_distance];
        distance = (distance_LSB | ((distance_MSB & 0x3F) << 8));
        
        // Process the intensity datum
        uint16_t offset_intensity = OFFSET_TO_4_DATA_READINGS + num_quad*N_DATA_QUADS + OFFSET_DATA_SIGNAL_LSB;
        uint8_t intensity_LSB = m_packet_full[offset_intensity];
        uint8_t intensity_MSB = m_packet_full[offset_intensity + 1];
        intensity = intensity_LSB | (intensity_MSB << 8); // Extract the signal strength of the measurement
    }
}

void Xv11Lidar::publishLaserScan()
{
    float motor_rpm_avg = m_motor_rpm_sum/m_num_good_readings;
    m_motor_rpm_sum = 0;
    m_num_good_readings = 0;
    if(motor_rpm_avg != 0)
    {
        m_laserscan_msg.time_increment = 1.0/(motor_rpm_avg*6); // 1/(revs/sec * 360 meas/rev)
    }

    m_laserscan_msg.header.seq = m_num_scans_rxd++;

    // Copy ranges and intensities to message
    for(int i=0; i<NUM_VALS_LASERSCAN; ++i)
    {
        m_laserscan_msg.ranges[i] = m_ranges[i];
        m_laserscan_msg.intensities[i] = m_intensities[i];
    }

    m_laserscan_idx = 0;
    m_laserscan_start = 1;
    m_laserscan_ready = 1;
    //pc.printf("Laserscan published.");
}

void Xv11Lidar::rxISR(void)
{
    const uint8_t rx_in = m_lidar_serial.getc();

    // Look for the start of the packet and advance to the next state if found
    if(m_state == STATE_FIND_COMMAND)
    {
        if(rx_in == COMMAND)
        {
            m_packet[m_packet_idx++] = rx_in;
            m_state++;
        }
    }
    // Keep building the packet, and process the packet if we've received enough bytes.
    else if(m_state == STATE_BUILD_PACKET)
    {
        m_packet[m_packet_idx++] = rx_in;
        if(m_packet_idx == PACKET_LENGTH) 
        {
            // Create a copy of the packet for use by the update function, reset variables,
            //     and start the search for the next packet
            for(int i=0; i<PACKET_LENGTH; ++i)
                m_packet_full[i] = m_packet[i];
                
            // Reset control variables
            m_packet_idx = 0;
            m_state = STATE_FIND_COMMAND;

            m_packet_ready = 1;
        }
    }
}