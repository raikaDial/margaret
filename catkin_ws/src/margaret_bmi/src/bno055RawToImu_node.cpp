////////////////////////////////////////////////////////////////////////////////////////////
/// @file bno055RawToImu_node.cpp
/// @brief Takes packets of raw IMU data from the BNO055 and converts them to Imu messages.
///
/// @author Ryker Dial
/// @date Feb 1, 2020
////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Imu.h>

class Bno055RawToImu
{
    public:
        Bno055RawToImu(ros::NodeHandle nh, std::string imu_raw_topic, std::string imu_topic, std::string frame_id)
            : m_nh(nh)
        {
            // Initialize static values in Imu message.
            // TODO: Set Imu covariance matrices
            m_imu_msg.header.frame_id = frame_id;

            // Configure publishers and subscribers
            m_imu_raw_sub = m_nh.subscribe(imu_raw_topic, 1, &Bno055RawToImu::rawIMUDataCallback, this);
            m_imu_pub = m_nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
        }

        void rawIMUDataCallback(const std_msgs::UInt8MultiArray::ConstPtr & raw_imu_data)
        {
            // First 12 bytes of message are the timestamp and sequence id
            uint32_t sec = 0;
            uint32_t nsec = 0;
            uint32_t seq = 0;
            for(int i=0; i<4; ++i)
            {
                sec |= (raw_imu_data->data[i] << 8*(3-i));
                nsec |= (raw_imu_data->data[4+i] << 8*(3-i));
                seq |= (raw_imu_data->data[8+i] << 8*(3-i));
            }
            m_imu_msg.header.stamp.sec = sec;
            m_imu_msg.header.stamp.nsec = nsec;
            m_imu_msg.header.seq = seq;

            // Remaining bytes are accels, angular rates, and orientation quaternion
            // Acceleration is 
            m_imu_msg.linear_acceleration.x = ((int16_t)((raw_imu_data->data[12] << 8) | raw_imu_data->data[13]))/100.0;
            m_imu_msg.linear_acceleration.y = ((int16_t)((raw_imu_data->data[14] << 8) | raw_imu_data->data[15]))/100.0;
            m_imu_msg.linear_acceleration.z = ((int16_t)((raw_imu_data->data[16] << 8) | raw_imu_data->data[17]))/100.0;

            // Angular rates are reported as rad/s * 900.
            m_imu_msg.angular_velocity.x = ((int16_t)((raw_imu_data->data[18] << 8) | raw_imu_data->data[19]))/900.0;
            m_imu_msg.angular_velocity.y = ((int16_t)((raw_imu_data->data[20] << 8) | raw_imu_data->data[21]))/900.0;
            m_imu_msg.angular_velocity.z = ((int16_t)((raw_imu_data->data[22] << 8) | raw_imu_data->data[23]))/900.0;

            // Quaternion is scaled by 2^13.
            m_imu_msg.orientation.x = ((int16_t)((raw_imu_data->data[24] << 8) | raw_imu_data->data[25]))/((double)(1<<14));
            m_imu_msg.orientation.y = ((int16_t)((raw_imu_data->data[26] << 8) | raw_imu_data->data[27]))/((double)(1<<14));
            m_imu_msg.orientation.z = ((int16_t)((raw_imu_data->data[28] << 8) | raw_imu_data->data[29]))/((double)(1<<14));
            m_imu_msg.orientation.w = ((int16_t)((raw_imu_data->data[30] << 8) | raw_imu_data->data[31]))/((double)(1<<14));

            m_imu_pub.publish(m_imu_msg);
        }

    private:
        ros::NodeHandle m_nh;
        ros::Subscriber m_imu_raw_sub;
        ros::Publisher m_imu_pub;
        sensor_msgs::Imu m_imu_msg;
};

int main(int argc, char** argv)
{
	// Setup the ROS node
	ros::init(argc, argv, "bno055raw2imu_node");
	ros::NodeHandle nh;

	// Start up the Imu message assembler.
    Bno055RawToImu bno055_raw_to_imu(nh, "imu_raw", "imu", "imu_link");
	ros::spin();

	return 0;
}