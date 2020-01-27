///////////////////////////////////////////////////////////////////////////
/// @file MbedHardwareEthernet.h
/// @brief Implements Network API for Rosserial
///
/// @author Ryker Dial
/// @date Jan. 26, 2020
///////////////////////////////////////////////////////////////////////////

#ifndef ROS_MBED_HARDWARE_ETHERNET_H_INCLUDED
#define ROS_MBED_HARDWARE_ETHERNET_H_INCLUDED

#include <mbed.h>
#include <EthernetInterface.h>
#include <platform/CircularBuffer.h>

#define RX_BUFF_SIZE 2048

extern Serial pc;

class MbedHardwareEthernet
{
    public:
        MbedHardwareEthernet() {}

        ~MbedHardwareEthernet()
        {
            delete m_server;
            delete m_eth;
            delete m_socket;
        }

        // Initialize the network interface and open our socket
        void init(char* server_ip)
        {
            // Store address of rosserial server
            m_server = new SocketAddress(server_ip, 11411);

            const char* rosserial_ip = m_server->get_ip_address();
            uint16_t rosserial_port = m_server->get_port();
            pc.printf("Rosserial IP: %s\n", rosserial_ip ? rosserial_ip : "None");
            pc.printf("Rosserial Port: %d\n", rosserial_port);

            // Spin up the network interface
            m_eth = new EthernetInterface();
            m_eth->connect();

            const char* ip = m_eth->get_ip_address();
            const char* netmask = m_eth->get_netmask();
            const char *gateway = m_eth->get_gateway();
            pc.printf("IP address: %s\n", ip ? ip : "None");
            pc.printf("Netmask: %s\n", netmask ? netmask : "None");
            pc.printf("Gateway: %s\n", gateway ? gateway : "None");    

            // Open the socket
            m_socket = new TCPSocket();
            m_socket->open(m_eth);

            m_socket->bind(11411);

            pc.printf("Connect socket.\n");
            int error = m_socket->connect(*m_server);
            if(error < 0)
            {
                pc.printf("Socket connect failed: %d", error);
            }
        }

        ///////////////////////////////////////////////////////////////////////////
        /// @brief Read a byte from the network socket if available.
        /// @return Byte of data if available, -1 otherwise.
        ///////////////////////////////////////////////////////////////////////////
        int read()
        {
            // Check if we have bytes to read from the socket.
            /// @todo Reading data from the socket should be done in a separate function.
            uint8_t rx_buff[64];
            m_socket->set_blocking(false);
            int rx_count = m_socket->recv(&rx_buff, 64);

            if(rx_count > 0)
            {
                //pc.printf("Rx Bytes: %d\n", rx_count);
                for(int i=0; i<rx_count; ++i)
                {
                    m_buff.push(rx_buff[i]);
                }
            }

            if(!m_buff.empty())
            {
                uint8_t data;
                m_buff.pop(data);
                return data;
            }
            else { return -1; }
        }

        ///////////////////////////////////////////////////////////////////////////
        /// @brief Write data to the network socket.
        ///
        /// @param data - pointer to array containing data to send.
        /// @param length - number of bytes of data to send.
        ///////////////////////////////////////////////////////////////////////////
        void write(uint8_t* data, int length)
        {
            m_socket->set_blocking(true);
            m_socket->send(data,length);
        }

        unsigned long time() { return HAL_GetTick(); }

    private:
        EthernetInterface* m_eth;
        TCPSocket* m_socket;
        SocketAddress* m_server;

        CircularBuffer<uint8_t, RX_BUFF_SIZE> m_buff;
};

#endif // ROS_MBED_HARDWARE_ETHERNET_H_INCLUDED