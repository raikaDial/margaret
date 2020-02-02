////////////////////////////////////////////////////////////////////////////////////////////
/// @file xv11RawToLaserScan_node.cpp
/// @brief Takes packets of raw data from an XV11 LIDAR and converts them to a LaserScan message.
///
/// @author Ryker Dial
/// @date Feb 1, 2020
////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#define _USE_MATH_DEFINES
#include <cmath>

class Xv11RawToLaserScan
{
    public:
        Xv11RawToLaserScan(ros::NodeHandle nh, std::string scan_raw_topic, std::string scan_topic, std::string frame_id)
            : m_nh(nh)
        {
            // Initialize static values of LaserScan message
            m_scan_msg.header.frame_id = frame_id;
            m_scan_msg.angle_increment = M_PI/180.0;
            m_scan_msg.range_min = 0.02;
            m_scan_msg.range_max = 6.0;

            // Configure publishers and subscribers
            m_scan_raw_sub = m_nh.subscribe(scan_raw_topic, 1, &Xv11RawToLaserScan::scanRawCallback, this);
            m_scan_pub = m_nh.advertise<sensor_msgs::LaserScan>(scan_topic, 1);
        }

        void scanRawCallback(const std_msgs::UInt16MultiArray::ConstPtr & raw_scan_data)
        {
            int num_vals_scan = (raw_scan_data -> data.size() - 9) / 2; // Get number of scan data points.
            
            m_scan_msg.ranges.resize(num_vals_scan);
		    m_scan_msg.intensities.resize(num_vals_scan);

            // First 12 bytes of message are the timestamp and sequence id
            m_scan_msg.header.stamp.sec = raw_scan_data->data[0] << 16 | raw_scan_data->data[1];
            m_scan_msg.header.stamp.nsec = raw_scan_data->data[2] << 16 | raw_scan_data->data[3];
            m_scan_msg.header.seq = raw_scan_data->data[4] << 16 | raw_scan_data->data[5];

            // Next 2 bytes are the starting angle of the scan.
            m_scan_msg.angle_min = raw_scan_data->data[6]*M_PI/180.0;
            m_scan_msg.angle_max = fmod(m_scan_msg.angle_min + m_scan_msg.angle_increment*(num_vals_scan-1), 2*M_PI);

            // Next 4 bytes are the average motor rpm during this scan, which we use to calculate the time increment.
            uint32_t motor_rpm_avg_bytes = raw_scan_data->data[7] << 16 | raw_scan_data->data[8];
            float motor_rpm_avg = *const_cast<const float*>(reinterpret_cast<const float*>(&motor_rpm_avg_bytes));
			if(motor_rpm_avg != 0) // Check just in case something weird happened
            { 
				m_scan_msg.time_increment = 1.0/(motor_rpm_avg*6); // (revs/second * 360 meas/rev)^(-1)
			}

            // Remaining bytes are the range and intensity measurements
		    for(int i=0; i<num_vals_scan; ++i)
            {
		    	float distance = (raw_scan_data->data[9+i])/1000.0; // convert to meters

		    	// If reported measurement is out of range, set distance and intensity to zero.
		    	if((distance < m_scan_msg.range_min) || (distance >  m_scan_msg.range_max)) {
		    		m_scan_msg.ranges[i] = 0;
		    		m_scan_msg.intensities[i] = 0;
		    	}
		    	else // Measurements are good, store them.
                { 
		    		m_scan_msg.ranges[i] = distance;
		    		m_scan_msg.intensities[i] = raw_scan_data->data[9 + num_vals_scan + i];
		    	}
		    }

            m_scan_pub.publish(m_scan_msg);        
        }

    private:
        ros::NodeHandle m_nh;
        ros::Subscriber m_scan_raw_sub;
        ros::Publisher m_scan_pub;
        sensor_msgs::LaserScan m_scan_msg;
};

int main(int argc, char** argv)
{
	// Setup the ROS node
	ros::init(argc, argv, "xv11RawToLaserScan_node");
	ros::NodeHandle nh;

    ros::NodeHandle pr_nh("~"); // Private node handle to get params
	std::string frame_id;
	if( !pr_nh.getParam("frame_id", frame_id) ) { frame_id = "laser_link"; }

	// Start up the LaserScan message assembler.
	Xv11RawToLaserScan xv11_raw_to_laserscan(nh, "scan_raw", "scan", frame_id);
	ros::spin();

	return 0;
}