#ifndef ROS_STM32F767ZI_HARDWARE_H_
#define ROS_STM32F767ZI_HARDWARE_H_

#include <LwIP.h> // LwIP automatically renews DHCP lease, so no need to do ourselves.
#include <STM32Ethernet.h>

class STM32F767ZIHardware 
{
  public:
    STM32F767ZIHardware(): m_ip(192,168,1,4), m_server(192,168,1,8), m_port(11411)
    {}

    void connect()
    {
      while(!m_eth.connected())
      {
        m_eth.connect(m_server, m_port);
      }      
    }

    void init()
    {
      Ethernet.begin(m_ip); 
      connect();
    }

    int read()
    {
      while(!m_eth.connected())
      {
        connect();
      }

      if(m_eth.available())
      {
        char data = m_eth.read();
        return data;
      }
      else
      {
        return -1;
      }
    }

    void write(uint8_t* data, int length)
    {
      while(!m_eth.connected())
      {
        connect();
      }

      m_eth.write(data, length);
    }

    unsigned long time(){ return millis(); }

  protected:
    EthernetClient m_eth;
    IPAddress m_ip;
    IPAddress m_server;
    int m_port;
};

#endif