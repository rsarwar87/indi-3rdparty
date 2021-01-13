#ifndef __PELTIER_HEADER__
#define __PELTIER_HEADER__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <assert.h> 
#include <array>
#include <algorithm>
#include "rpigpio_trigger.hpp"
#include <arpa/inet.h>
#include <cpr/cpr.h>
#include <chrono>
#include <thread>

using namespace std;

class PeltierTrigger
{
  public:
  PeltierTrigger (std::string port)
  {
    SetPort(port);
  }

  ~PeltierTrigger()
  {
    ptr_gpio.reset();
  }

  bool SetPort(std::string port){
    ptr_gpio.reset();
    is_esp = false;
    is_pi = false;
    is_http = false;
    is_connected = true;
    if (port.size() == 0) return false;
    if(isPiTrigger(port))
    {
      is_pi = true;
      return true;
    }
    if (SetEspIP(port))
    {
      is_esp = true;
      return true;
    }
    is_connected = false;
    return false;
  }

  bool start_cooling()
  {
    if (is_pi)
      ptr_gpio->SetGpio();
    else if (is_esp & !is_http)
    {
      return sendTCPCommand("\xA0\x01\x01\xA2");
    }
    else if (is_esp)
    {
      cpr::Response res = cpr::Get(cpr::Url{"http://" + esp_ip +"/RELAY=ON"});
      if (res.status_code != 200) return false;
    }
    else
      return false;
    return true;
  }
  bool stop_cooling()
  {
    if (is_pi)
      ptr_gpio->ClearGpio();
    else if (is_esp & !is_http)
    {
      return sendTCPCommand("\xA0\x01\x00\xA1");
    }
    else if (is_esp)
    {
      cpr::Response res = cpr::Get(cpr::Url{"http://" + esp_ip +"/RELAY=OFF"});
      if (res.status_code != 200) return false;
    }
    else
      return false;
    return true;
  }

  bool get_is_connected(){
    return is_connected;
  }

  private:
  std::unique_ptr<PiGpioWrapper> ptr_gpio;
  bool is_esp;
  bool is_http;
  bool is_pi;
  bool is_connected;
  std::string esp_ip;

  bool SetEspIP(std::string ip){
    struct sockaddr_in sa;
    int result = inet_pton(AF_INET, ip.c_str(), &(sa.sin_addr));
    if (result == 0)
    {
      return false;
    }
    esp_ip = ip;
    cpr::Response res = cpr::Get(cpr::Url{"http://" + esp_ip});
    is_http = (res.status_code == 200);
    return true;
  }
  bool isPiTrigger(std::string port){
    size_t n = std::count(port.begin(), port.end(), '@');
    size_t n2 = std::count(port.begin(), port.end(), ':');
    size_t found = port.find('@');
    if (n != 1 || found != 3 || port.size() - found > 3 || n2 != 0) 
    {
      return false;
    }
    port.erase(0, found + 1);
		if (port.find_first_not_of("0123456789") != std::string::npos)
		{
      return false;

		}
		ptr_gpio = make_unique<PiGpioWrapper>(port,true);
    return true;
  }


  bool sendTCPCommand(const char* val)
  {
    int sock = 0; 
    struct sockaddr_in serv_addr; 
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("\n Socket creation error \n"); 
        return false; 
    } 
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(8080); 
    if(inet_pton(AF_INET, esp_ip.c_str(), &serv_addr.sin_addr)<=0)  
    { 
        printf("\nInvalid address/ Address not supported \n"); 
        return false; 
    } 

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("\nConnection Failed \n"); 
        return false; 
    } 
    send(sock , val, 4, 0); 
    close(sock);
    return true;
  }


};
#if 0
int main()
{

	PeltierTrigger dut("192.168.1.144");
    std::cout << "open " <<  std::endl;
  dut.start_cooling();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "close " <<  std::endl;
  dut.stop_cooling();

}
#endif
#endif
