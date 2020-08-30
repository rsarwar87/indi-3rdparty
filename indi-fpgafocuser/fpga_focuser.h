/*******************************************************************************
  Copyright(c) 2014 Radek Kaczorek  <rkaczorek AT gmail DOT com>

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#ifndef FOCUSRPI_H
#define FOCUSRPI_H

#include <fpgafocuser.hpp>
#include <indifocuser.h>
#include <arpa/inet.h>
#include <string.h>

class FpgaFocuser : public INDI::Focuser
{
public:
	FpgaFocuser();
	virtual ~FpgaFocuser();
	const char *getDefaultName(){ return (char *)"Fpga-Koheron Focuser"; };
	virtual bool initProperties();
	virtual bool updateProperties();
	virtual void ISGetProperties (const char *dev);
	virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
	virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);
	virtual bool ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n);
	virtual bool ISSnoopDevice(XMLEle *root);
	static void updateStatusHelper(void *context);
	static void updateTemperatureHelper(void *context);
	static void temperatureCompensationHelper(void *context);
  bool SyncFocuser( uint32_t ticks ) override;
protected:
	virtual IPState MoveAbsFocuser(int ticks);
	virtual IPState MoveRelFocuser(FocusDirection dir, int ticks);
	virtual bool saveConfigItems(FILE *fp);
	virtual bool ReverseFocuser(bool enabled);
	virtual bool AbortFocuser();
  virtual bool SetFocuserBacklash(int32_t steps);
  virtual bool SetFocuserBacklashEnabled(bool enabled);
  virtual bool SetFocuserSpeed(int speed);
	virtual void TimerHit();
private:
	virtual bool Connect();
	virtual bool Disconnect();
	virtual int savePosition(int pos);
	virtual bool readtemp();

	IText IPAddress[1];
	ITextVectorProperty IPAddressP;
	IText ActiveTelescopeT[1];
	ITextVectorProperty ActiveTelescopeTP;
	ISwitchVectorProperty MotorBoardSP;

	ISwitch ResetAbsPosS[1];
	ISwitchVectorProperty ResetAbsPosSP;
	ISwitch TemperatureCompensateS[2];
	ISwitchVectorProperty TemperatureCompensateSP;

	INumber FocuserInfoN[3];
	INumberVectorProperty FocuserInfoNP;
	INumber FocusBacklashPeriodN[1];
	INumberVectorProperty FocusBacklashNP;
	INumber FocuserTravelN[1];
	INumberVectorProperty FocuserTravelNP;
	INumber ScopeParametersN[2];
	INumberVectorProperty ScopeParametersNP;

	INumber FocusTemperatureN[1];
	INumberVectorProperty FocusTemperatureNP;
	INumber TemperatureCoefN[1];
	INumberVectorProperty TemperatureCoefNP;

	double motorPeriodUs;
	float lastTemperature;
	bool reverse_direction = false;

	void updateStatus();
	void updateStatusFunc();
	int updateStatusID { -1 };
	void getFocuserInfo();
	int updateTemperatureID { -1 };
	void updateTemperature();
	int temperatureCompensationID { -1 };
	void temperatureCompensation();

  bool hw_is_initialized {false};
  bool hw_is_running {false};
  bool hw_direction {false};
  bool detected_motion {false};

  bool validateIpAddress(const std::string &ipAddress)
  {
    struct sockaddr_in sa;
    int result = inet_pton(AF_INET, ipAddress.c_str(), &(sa.sin_addr));
    return result != 0;
  }

  bool is_number(const std::string& s)
  {
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
  }
  std::unique_ptr<indi_focuser_interface> koheron_interface;
};

#endif
