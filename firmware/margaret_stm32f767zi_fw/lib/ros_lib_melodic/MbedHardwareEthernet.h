// @file MbedHardwareEthernet.h
// @author Ryker Dial
// @date Jan. 26, 2020
// @brief Implements Network API for Rosserial

#ifndef ROS_MBED_HARDWARE_ETHERNET_H_
#define ROS_MBED_HARDWARE_ETHERNET_H_

#include <mbed.h>
#include <EthernetInterface.h>
#include <platform/CircularBuffer.h>

#define RX_BUFF_SIZE 2048

extern Serial pc;
extern DigitalOut led2;
extern DigitalOut led3;

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

        int read()
        {
            led2=1;
            //if(m_socket->connect(*m_server) > 0)
            {
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
                //m_socket->close();
            }
            led2=0; 

            if(!m_buff.empty())
            {
                uint8_t data;
                m_buff.pop(data);
                return data;
            }
            else { return -1; }
        }

        void write(uint8_t* data, int length)
        {
            led3 = 1;
            //if(m_socket->connect(*m_server) > 0)
            {
                //pc.printf("Tx Bytes: %d\n", length);
                m_socket->set_blocking(true);
                m_socket->send(data,length);
                //m_socket->close();
            }
            led3 = 0;
        }

        unsigned long time() { return HAL_GetTick(); }

    private:
        EthernetInterface* m_eth;
        TCPSocket* m_socket;
        SocketAddress* m_server;

        CircularBuffer<uint8_t, RX_BUFF_SIZE> m_buff;
};

#endif /* ROS_MBED_HARDWARE_ETHERNET_H_ */

// #define RX_BUFF_SIZE 2048

// extern Serial pc;

// extern DigitalOut led2;
// extern DigitalOut led3;

// class MbedHardwareEthernet {
//   public:
//     MbedHardwareEthernet()
//       : m_port(11411), m_server("192.168.1.5", 11411)
//     {
//     }
  
//     ~MbedHardwareEthernet()
//     {
//         m_sock->close();
//         m_eth->disconnect();

//         delete m_eth;
//         delete m_sock;
//     }
    
//     void init()
//     {
//         init("192.168.1.255");
//     }
          //  void handleSocketSigio(EventFlags *evt, TCPSocket *socke
//     void init(char* server_ip)
//     {
//         nsapi_error_t error;

//         m_server_ip = server_ip;
//         m_eth = new EthernetInterface();
//         error = m_eth->connect();
//         if(error < 0)
//         {
//           pc.printf("Error connecting to network interface: %d\n", error);
//           return;
//         }

//         const char* ip = m_eth->get_ip_address();
//         const char* netmask = m_eth->get_netmask();
//         const char *gateway = m_eth->get_gateway();
//         pc.printf("IP address: %s\n", ip ? ip : "None");
//         pc.printf("Netmask: %s\n", netmask ? netmask : "None");
//         pc.printf("Gateway: %s\n", gateway ? gateway : "None");    

//         m_sock = new TCPSocket();
//         //m_sock = new UDPSocket();
//         //m_sock->set_blocking(false); // Set to non-blocking so we don't wait for data.

//         error = m_sock->open(m_eth);
//         if(error < 0)
//         {
//           pc.printf("Error opening socket: %d\n", error);
//           m_eth->disconnect();
//           return;
//         }

//         error = m_sock->bind("0.0.0.0", 11411);
//         if(error < 0)
//         {
//             pc.printf("Error binding socket: %d\n", error);
//             m_sock->close();
//             m_eth->disconnect();
//             return;

//         }
//         m_sock->attach(callback(this, &MbedHardwareEthernet::socketRxISR));  

//         //SocketAddress m_server("192.168.1.5",11411);

//         pc.printf("Connect socket.\n");
//         error = m_sock->connect(m_server);
//         if(error < 0)
//         {
//             pc.printf("Socket connect failed: %d", error);
//         }
//         // wait(1.0);
//     }

//     int read(){
//         //nsapi_error_t error;
//         //error = m_sock->connect(m_server_ip, 11411);
//         // if(error < 0)
//         // {
//         //     pc.printf("Socket connect failed in read: %d", error);
//         // }
//         //pc.printf("Eth Read");
//         /*uint8_t rx_in; //[64];
//         m_sock->set_blocking(false);
//         int rx_count = m_sock->recv(&rx_in, 1);*/

//         // uint8_t rx_in[64];
//         // m_sock->set_blocking(false);
//         // SocketAddress m_server("192.168.1.5",11411);

//         // int rx_count = m_sock->recvfrom(&m_server, &rx_in, 64);

//         // //pc.printf("rx count: %d", rx_count);

//         // if(rx_count > 0)
//         // {
//         //     for(int i=0; i<rx_count; ++i)
//         //     {
//         //         m_buff.push(rx_in[i]);
//         //     }
//         // }

//         if(m_buff.empty())
//         {
//             return -1;
//         }
//         else
//         {
//             uint8_t data;
//             m_buff.pop(data);
//             return data;
//         }
//         //return -1;
//     }
//     void write(uint8_t* data, int length)
//     {
//         led3=1;
//         nsapi_error_t error;

//         pc.printf("Write: %d\n", length);

//         //m_sock->sendto("192.168.1.255", 11411, data, length);
//         //SocketAddress m_server("192.168.1.5",11411);
//         //m_sock->set_blocking(true);
//         m_sock->sendto(m_server, data, length);
//         //error = m_sock->connect(m_server_ip, 11411);
//         // if(error < 0)
//         // {
//         //     pc.printf("Socket connect failed in write: %d", error);
//         // }
//         //pc.printf("Eth Write");
//         // if(!m_sock->send(data, length))
//         // {
//         //     pc.printf("Error sending data");
//         // }
//         led3=0;
//     }

//     unsigned long time(){return t.read_ms();}

//     void socketRxISR()
//     {
//         triggered_int=true;
//         led2=1;
//         // uint8_t rx_in[64];

//         // SocketAddress m_server("192.168.1.5",11411);
//         // m_sock->set_blocking(false);
//         // int rx_count = 0;
//         // while(1)
//         // {
//         //     rx_count = m_sock->recvfrom(&m_server, &rx_in, 64);

//         //     if(rx_count > 0) {
//         //         for(int i=0; i<rx_count; ++i)
//         //         {
//         //             m_buff.push(rx_in[i]);
//         //             uint8_t data;
//         //             //m_buff.pop(data);

//         //         }
//         //     }
//         //     else
//         //     {
//         //         break;
//         //     }
//         // }
//         led2=0;    
//     }

// //protected:
//     EthernetInterface* m_eth;
//     TCPSocket* m_sock;
//     //UDPSocket* m_sock;
//     SocketAddress m_server;

//     CircularBuffer<uint8_t, RX_BUFF_SIZE> m_buff;

//     bool triggered_int;
//     char* m_server_ip;
//     int m_port;
//     Timer t;
// };


