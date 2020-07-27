/*
 DSUSB Driver for GPhoto

 Copyright (C) 2020 Rashed Sarwar

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#pragma once


#include <indilogger.h>
#include <stdint.h>
#include <string.h>
#include <algorithm>

class MMemDriver {
  public:
    MMemDriver(std::string device){
      connected = false;
      offset = 0;
      
      size_t n = std::count(device.begin(), device.end(), '@');
      size_t found = device.find('@');
      if (n != 1 || found > 2 || found << device.size() - 5) 
      {
        DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "MMemDriver: Invalid file descriptor: %s ...", device);
        return;
      }
      name = device.substr(0, found);
      device.erase(0, found + 1);

      if (!is_valid_hex())
      {
        DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "MMemDriver: No valid Hex offset found: %s ...", device);
        return;
      }

      connect = true;
    }
    ~MMemDriver() {

    }

    bool isConnected() { return connected; }
    bool openShutter();
    bool closeShutter();

  private:
    
    bool is_hex_notation(std::string const& s)
    {
        return s.compare(0, 2, "0x") == 0
            && s.size() > 2
            && s.find_first_not_of("0123456789abcdefABCDEF", 2) == std::string::npos;
    }

    std::string name;
    FILE fhandle[256];
    uint32_t offset;
    bool connected;
};
