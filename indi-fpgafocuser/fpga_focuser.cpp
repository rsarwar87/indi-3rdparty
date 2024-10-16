/*******************************************************************************
  Copyright(c) 2014-2020 Radek Kaczorek  <rkaczorek AT gmail DOT com>

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

/*
 * TO DO:
 * - Add temperature compensation auto learning and save temperature compensation curve to xml
 */

#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <math.h>
#include <memory>
#include "config.h"

#include "fpga_focuser.h"
#include <connectionplugins/connectiontcp.h>

// We declare an auto pointer to FpgaFocuser.
std::unique_ptr<FpgaFocuser> fpgaFocuser(new FpgaFocuser());

#define STATUS_UPDATE_TIMEOUT (2 * 1000) // 60 sec
#define TEMPERATURE_UPDATE_TIMEOUT (60 * 1000) // 60 sec
#define TEMPERATURE_COMPENSATION_TIMEOUT (60 * 1000) // 60 sec

void ISPoll(void *p);


void ISInit()
{
	static int isInit = 0;

	if (isInit == 1)
	return;
	if(fpgaFocuser.get() == 0)
	{
	isInit = 1;
	fpgaFocuser.reset(new FpgaFocuser());
	}
}

void ISGetProperties(const char *dev)
{
        ISInit();
        fpgaFocuser->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
        ISInit();
        fpgaFocuser->ISNewSwitch(dev, name, states, names, num);
}

void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int num)
{
        ISInit();
        fpgaFocuser->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
        ISInit();
        fpgaFocuser->ISNewNumber(dev, name, values, names, num);
}

void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
	INDI_UNUSED(dev);
	INDI_UNUSED(name);
	INDI_UNUSED(sizes);
	INDI_UNUSED(blobsizes);
	INDI_UNUSED(blobs);
	INDI_UNUSED(formats);
	INDI_UNUSED(names);
	INDI_UNUSED(n);
}

void ISSnoopDevice (XMLEle *root)
{
	fpgaFocuser->ISSnoopDevice(root);
}

FpgaFocuser::FpgaFocuser()
{
	setVersion(VERSION_MAJOR,VERSION_MINOR);
	FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_REVERSE | FOCUSER_CAN_ABORT 
      | FOCUSER_CAN_SYNC | FOCUSER_HAS_VARIABLE_SPEED | FOCUSER_HAS_BACKLASH);
	Focuser::setSupportedConnections(CONNECTION_TCP);
}

FpgaFocuser::~FpgaFocuser()
{
	// delete properties independent of connection status
  koheron_interface.reset();
}

bool FpgaFocuser::SetFocuserMaxPosition(uint32_t value)
{
  if (FocusAbsPosN[0].value > value) 
  {
    koheron_interface->SetFocuserPosition(value);
    updateStatusFunc();
  }
  if (!koheron_interface->SetGridPerRevolution(value))
  {
	  DEBUG(INDI::Logger::DBG_WARNING, "Failed to set max position.");
    return false;
  }
	FocusMaxPosN[0].value = value;
	//FocusMaxPosN[0].min = value;
	//FocusMaxPosN[0].max = value;
	FocusRelPosN[0].max = value/2;
	FocusRelPosN[0].step = value/100;
	FocusRelPosN[0].min = 0;
	FocusBacklashN[0].max = value;
	FocusAbsPosN[0].max = FocusSyncN[0].max = value;
	FocusAbsPosN[0].min = FocusSyncN[0].min = 0;
	FocusAbsPosN[0].step = FocusSyncN[0].step = value/100;
  IUUpdateMinMax(&FocusAbsPosNP);
  IUUpdateMinMax(&FocusRelPosNP);
  IUUpdateMinMax(&FocusSyncNP);
	getFocuserInfo();
	DEBUGF(INDI::Logger::DBG_DEBUG, "Max position set to %d.", value);
  return true;
}

bool FpgaFocuser::Connect()
{
  const char * ip = tcpConnection->host();
  if (ip == nullptr )
  {
      LOG_ERROR("Error! Server address is missing or invalid.");
      return false;
  }
  uint32_t port = tcpConnection->port(); 
  if (!validateIpAddress(ip))
  {
			DEBUGF(INDI::Logger::DBG_ERROR, "Invalid IP address: %si:%d", ip, port);
			return false;
  }

  try {
      koheron_interface = std::make_unique<indi_focuser_interface>(ip, port);
  }
  catch (const std::exception& e)
  {
			DEBUGF(INDI::Logger::DBG_ERROR, "Could not connect to server: %s:%d %s"
          , ip, port, e.what());
			return false;
  }
  updateStatusFunc();

  if (!hw_is_initialized) 
    koheron_interface->Initialize();
	motorPeriodUs = 1000000.0 / koheron_interface->GetTimerInterruptFreq();
	FocusSpeedN[0].min = koheron_interface->get_minimum_period()*3*motorPeriodUs;
	FocusSpeedN[0].max = koheron_interface->get_maximum_period()*motorPeriodUs;
	FocusBacklashPeriodN[0].min = FocusSpeedN[0].min;
	FocusBacklashPeriodN[0].max = FocusSpeedN[0].max;

  FocusMaxPosN[0].max = 0x3FFFFFFF;
  //SetFocuserMaxPosition(koheron_interface->GetGridPerRevolution());
  //if (!hw_is_initialized)
  if (savePosition(-1) != -1)
  {
	  uint32_t val = (int) savePosition(-1);
    if (val > koheron_interface->GetGridPerRevolution())
    {
			DEBUG(INDI::Logger::DBG_WARNING, "Discarding stored AbsPosition as it is greater"
          "than FocusAbsPosN.max");
      FocusAbsPosN[0].value = koheron_interface->GetFocuserPosition();
    }
    else
    {
	    FocusAbsPosN[0].value = (int) savePosition(-1);
      koheron_interface->SetFocuserPosition(FocusAbsPosN[0].value);
			DEBUGF(INDI::Logger::DBG_WARNING, "Assigning new FocusAbsPosN %d", (int)FocusAbsPosN[0].value);
    }
  }
  else
  {
			DEBUG(INDI::Logger::DBG_WARNING, "FocusAbsPosN not found, retreiving ..");
	    FocusAbsPosN[0].value = koheron_interface->GetFocuserPosition();
  }

	updateStatusID = IEAddTimer(STATUS_UPDATE_TIMEOUT, updateStatusHelper, this);
	// Update focuser parameters
	getFocuserInfo();

	DEBUG(INDI::Logger::DBG_SESSION, "Fpga Focuser connected successfully.");

	return true;
}

bool FpgaFocuser::Disconnect()
{
	// Close device
	IERmTimer(updateStatusID);
  koheron_interface.reset();


	// Unlock BCM Pins setting

	// Stop timers
	IERmTimer(updateTemperatureID);
	IERmTimer(temperatureCompensationID);

	DEBUG(INDI::Logger::DBG_SESSION, "Astroberry Focuser disconnected successfully.");

	return true;
}

bool FpgaFocuser::initProperties()
{
	INDI::Focuser::initProperties();

  const char * TEMP_TAB = "Temperature Compensation";
	// Backlash setting
	IUFillNumber(&FocusBacklashPeriodN[0], "FOCUS_BACKLASH_PERIOD_VALUE", "period (microseconds)", "%0.0f", 15, 19000, 1, 45);
	IUFillNumberVector(&FocusBacklashPeriodNP, FocusBacklashPeriodN, 1, getDeviceName(), "FOCUS_BACKLASH", "Backlash Period", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);
	// Focuser Info
	IUFillNumber(&FocuserInfoN[0], "CFZ_STEP_ACT", "Step Size (μm)", "%0.2f", 0, 1000, 1, 0);
	IUFillNumber(&FocuserInfoN[1], "CFZ", "Critical Focus Zone (μm)", "%0.2f", 0, 1000, 1, 0);
	IUFillNumber(&FocuserInfoN[2], "STEPS_PER_CFZ", "Steps / Critical Focus Zone", "%0.0f", 0, 1000, 1, 0);
	IUFillNumberVector(&FocuserInfoNP, FocuserInfoN, 3, getDeviceName(), "FOCUSER_PARAMETERS", "Focuser Info", TEMP_TAB, IP_RO, 0, IPS_IDLE);


	// Reset absolute possition
	IUFillSwitch(&ResetAbsPosS[0],"RESET_ABS","GoToHome",ISS_OFF);
	IUFillSwitchVector(&ResetAbsPosSP,ResetAbsPosS,1,getDeviceName(),"RESET_ABS_SW","Reset Position",MAIN_CONTROL_TAB,IP_RW,ISR_1OFMANY,0,IPS_IDLE);


	// Active telescope setting
	IUFillText(&ActiveTelescopeT[0], "ACTIVE_TELESCOPE_NAME", "Telescope", "Telescope Simulator");
	IUFillTextVector(&ActiveTelescopeTP, ActiveTelescopeT, 1, getDeviceName(), "ACTIVE_TELESCOPE", "Snoop devices", TEMP_TAB,IP_RW, 0, IPS_IDLE);

	// Maximum focuser travel
	IUFillNumber(&FocuserTravelN[0], "FOCUSER_TRAVEL_VALUE", "mm", "%0.0f", 10, 200, 10, 10);
	IUFillNumberVector(&FocuserTravelNP, FocuserTravelN, 1, getDeviceName(), "FOCUSER_TRAVEL", "Max Travel", TEMP_TAB, IP_RW, 0, IPS_IDLE);

	// Snooping params
	IUFillNumber(&ScopeParametersN[0], "TELESCOPE_APERTURE", "Aperture (mm)", "%g", 10, 5000, 0, 0.0);
	IUFillNumber(&ScopeParametersN[1], "TELESCOPE_FOCAL_LENGTH", "Focal Length (mm)", "%g", 10, 10000, 0, 0.0);
	IUFillNumberVector(&ScopeParametersNP, ScopeParametersN, 2, ActiveTelescopeT[0].text, "TELESCOPE_INFO", "Scope Properties", TEMP_TAB, IP_RW, 60, IPS_OK);

	// Focuser temperature
	IUFillNumber(&FocusTemperatureN[0], "FOCUS_TEMPERATURE_VALUE", "°C", "%0.2f", -50, 50, 1, 0);
	IUFillNumberVector(&FocusTemperatureNP, FocusTemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", TEMP_TAB, IP_RO, 0, IPS_IDLE);

	// Temperature Coefficient
	IUFillNumber(&TemperatureCoefN[0], "μm/m°C", "", "%.1f", 0, 50, 1, 0);
	IUFillNumberVector(&TemperatureCoefNP, TemperatureCoefN, 1, getDeviceName(), "Temperature Coefficient", "", TEMP_TAB, IP_RW, 0, IPS_IDLE);

	// Compensate for temperature
	IUFillSwitch(&ServerDebugrS[0], "Enable", "", ISS_OFF);
	IUFillSwitch(&ServerDebugrS[1], "Disable", "", ISS_ON);
	IUFillSwitchVector(&ServerDebugSP, ServerDebugrS, 2, getDeviceName(), "Enable SysLog Debugger in koheron server", "", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);
	// Compensate for temperature
	IUFillSwitch(&MotorTypeS[0], "DRV8825", "", ISS_ON);
	IUFillSwitch(&MotorTypeS[1], "TMC2226", "", ISS_OFF);
	IUFillSwitchVector(&MotorTypeSP, MotorTypeS, 2, getDeviceName(), "Switch between DRV8825 and TMC2226", "", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Compensate for temperature
	IUFillSwitch(&TemperatureCompensateS[0], "Enable", "", ISS_OFF);
	IUFillSwitch(&TemperatureCompensateS[1], "Disable", "", ISS_ON);
	IUFillSwitchVector(&TemperatureCompensateSP, TemperatureCompensateS, 2, getDeviceName(), "Temperature Compensate", "", TEMP_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Source for temperature
	IUFillSwitch(&TemperatureSensorS[0], "Pi-1wire (DS18B20)", "", ISS_ON);
	IUFillSwitch(&TemperatureSensorS[1], "FPGA (pin 14)", "", ISS_OFF);
	IUFillSwitch(&TemperatureSensorS[2], "FPGA (pin 15)", "", ISS_OFF);
	IUFillSwitchVector(&TemperatureSensorSP, TemperatureSensorS, 3, getDeviceName(), "Temperature Sensor", "", TEMP_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// initial values at resolution 1/1
	FocusMaxPosN[0].min = 1000;
	FocusMaxPosN[0].max = 100000;
	FocusMaxPosN[0].step = 1000;
	FocusMaxPosN[0].value = 10000;

	FocusRelPosN[0].min = 0;
	FocusRelPosN[0].max = 1000;
	FocusRelPosN[0].step = 100;
	FocusRelPosN[0].value = 100;

	FocusAbsPosN[0].min = 0;
	FocusAbsPosN[0].max = FocusMaxPosN[0].value;
	FocusAbsPosN[0].step = (int) FocusAbsPosN[0].max / 100;

	FocusMotionS[FOCUS_OUTWARD].s = ISS_ON;
	FocusMotionS[FOCUS_INWARD].s = ISS_OFF;
	IDSetSwitch(&FocusMotionSP, nullptr);

	// Add default properties
	// addAuxControls(); // use instead if simulation mode is added to code
	addDebugControl ();
	addConfigurationControl();
	removeProperty("POLLING_PERIOD", nullptr);

	return true;
}

void FpgaFocuser::ISGetProperties (const char *dev)
{
	INDI::Focuser::ISGetProperties(dev);
	return;
}

bool FpgaFocuser::updateProperties()
{
	INDI::Focuser::updateProperties();

	if (isConnected())
	{
		defineProperty(&ActiveTelescopeTP);
		defineProperty(&FocuserTravelNP);
		defineProperty(&FocusMotionSP);
		defineProperty(&FocuserInfoNP);
		defineProperty(&FocusBacklashPeriodNP);
		defineProperty(&ResetAbsPosSP);
		defineProperty(&ServerDebugSP);
		defineProperty(&MotorTypeSP);

		IDSnoopDevice(ActiveTelescopeT[0].text, "TELESCOPE_INFO");

		if (readtemp())
		{
			defineProperty(&FocusTemperatureNP);
			defineProperty(&TemperatureCoefNP);
			defineProperty(&TemperatureCompensateSP);
			defineProperty(&TemperatureSensorSP);
			readtemp(); // update immediately
			lastTemperature = FocusTemperatureN[0].value; // init last temperature
			IERmTimer(updateTemperatureID);
			updateTemperatureID = IEAddTimer(TEMPERATURE_UPDATE_TIMEOUT, updateTemperatureHelper, this); // set temperature update timer
			IERmTimer(temperatureCompensationID);
			temperatureCompensationID = IEAddTimer(TEMPERATURE_COMPENSATION_TIMEOUT, temperatureCompensationHelper, this); // set temperature compensation timer
		}

	} else {
		deleteProperty(ActiveTelescopeTP.name);
		deleteProperty(FocuserTravelNP.name);
		deleteProperty(FocusMotionSP.name);
		deleteProperty(FocuserInfoNP.name);
		deleteProperty(FocusBacklashPeriodNP.name);
		deleteProperty(ResetAbsPosSP.name);
		deleteProperty(FocusTemperatureNP.name);
		deleteProperty(TemperatureCoefNP.name);
		deleteProperty(TemperatureCompensateSP.name);
		deleteProperty(MotorTypeSP.name);
		deleteProperty(ServerDebugSP.name);
		deleteProperty(TemperatureSensorSP.name);
	}

	return true;
}

bool FpgaFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    // first we check if it's for our device
    if (!strcmp(dev, getDeviceName()))
    {
        // handle focus absolute position
        if (!strcmp(name, FocusAbsPosNP.name))
        {
            int newPos = (int)values[0];

            if (MoveAbsFocuser(newPos) == IPS_OK)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        // handle focus relative position
        if (!strcmp(name, FocusRelPosNP.name))
        {
            if (hw_is_running) return false;
            IUUpdateNumber(&FocusRelPosNP, values, names, n);
            IDSetNumber(&FocusRelPosNP, nullptr);

            //FOCUS_INWARD
            if (FocusMotionS[0].s == ISS_ON)
                FocusRelPosNP.s = MoveRelFocuser(reverse_direction ? FOCUS_OUTWARD : FOCUS_INWARD, FocusRelPosN[0].value);

            //FOCUS_OUTWARD
            if (FocusMotionS[1].s == ISS_ON)
                FocusRelPosNP.s = MoveRelFocuser(reverse_direction ? FOCUS_INWARD : FOCUS_OUTWARD, FocusRelPosN[0].value);

            return true;
        }

        // handle focus backlash
        if (!strcmp(name, FocusBacklashPeriodNP.name))
        {
            IUUpdateNumber(&FocusBacklashPeriodNP, values, names, n);
            FocusBacklashPeriodNP.s = IPS_BUSY;
            IDSetNumber(&FocusBacklashPeriodNP, nullptr);
	          if (!koheron_interface->set_backlash_period(FocusBacklashPeriodN[0].value/motorPeriodUs)) {
	            DEBUG(INDI::Logger::DBG_WARNING, "Failed to set backlash period.");
              return false;
            }
	          if (IUFindOnSwitchIndex(&FocusBacklashSP) == DefaultDevice::INDI_ENABLED) koheron_interface->enable_backlash(true);
            FocusBacklashPeriodNP.s = IPS_OK;
            IDSetNumber(&FocusBacklashPeriodNP, nullptr);
            DEBUGF(INDI::Logger::DBG_SESSION, "Backlash set to %0.0f steps and a period of %0.0f us.", FocusBacklashN[0].value, FocusBacklashPeriodN[0].value);
            return true;
        }

        // handle focuser travel
        if (!strcmp(name, FocuserTravelNP.name))
        {
            IUUpdateNumber(&FocuserTravelNP, values, names, n);
            getFocuserInfo();
            FocuserTravelNP.s = IPS_OK;
            IDSetNumber(&FocuserTravelNP, nullptr);
            DEBUGF(INDI::Logger::DBG_SESSION, "Maximum focuser travel set to %0.0f mm", FocuserTravelN[0].value);
            return true;
        }

        // handle temperature coefficient
        if (!strcmp(name, TemperatureCoefNP.name))
        {
            IUUpdateNumber(&TemperatureCoefNP, values, names, n);
            TemperatureCoefNP.s = IPS_OK;
            IDSetNumber(&TemperatureCoefNP, nullptr);
            DEBUGF(INDI::Logger::DBG_SESSION, "Temperature coefficient set to %0.1f μm/m°C", TemperatureCoefN[0].value);
            return true;
        }
    }

    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

bool FpgaFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    // first we check if it's for our device
    if (!strcmp(dev, getDeviceName()))
    {
        // handle focus presets
        if (!strcmp(name, PresetGotoSP.name))
        {
            IUUpdateSwitch(&PresetGotoSP, states, names, n);
            PresetGotoSP.s = IPS_BUSY;
            IDSetSwitch(&PresetGotoSP, nullptr);

            //Preset 1
            if (PresetGotoS[0].s == ISS_ON)
                MoveAbsFocuser(PresetN[0].value);

            //Preset 2
            if (PresetGotoS[1].s == ISS_ON)
                MoveAbsFocuser(PresetN[1].value);

            //Preset 3
            if (PresetGotoS[2].s == ISS_ON)
                MoveAbsFocuser(PresetN[2].value);

            PresetGotoS[0].s = ISS_OFF;
            PresetGotoS[1].s = ISS_OFF;
            PresetGotoS[2].s = ISS_OFF;
            PresetGotoSP.s   = IPS_OK;

            IDSetSwitch(&PresetGotoSP, nullptr);

            return true;
        }

        // handle reset absolute position
        if (!strcmp(name, ResetAbsPosSP.name))
        {
            IUResetSwitch(&ResetAbsPosSP);

            //set absolute position to zero and save to file
            MoveAbsFocuser(koheron_interface->GetFocuserHomePosition());
            IDSetNumber(&FocusAbsPosNP, nullptr);
            savePosition(0);

            DEBUG(INDI::Logger::DBG_SESSION, "Absolute Position reset to 0.");

            ResetAbsPosSP.s = IPS_IDLE;
            IDSetSwitch(&ResetAbsPosSP, nullptr);
            return true;
        }
        // handle temperature compensation
        if (!strcmp(name, TemperatureSensorSP.name))
        {
            IUUpdateSwitch(&TemperatureSensorSP, states, names, n);

            if (TemperatureSensorS[0].s == ISS_ON)
            {
                if (!temperatureCompensationID)
                    temperatureCompensationID =
                        IEAddTimer(TEMPERATURE_COMPENSATION_TIMEOUT, temperatureCompensationHelper, this);
                TemperatureSensorSP.s = IPS_OK;
                DEBUG(INDI::Logger::DBG_SESSION, "Temperature is sourced from DS18B20 connected to the RPi 1-wire pin");
            }

            if (TemperatureSensorS[1].s == ISS_ON)
            {
                IERmTimer(temperatureCompensationID);
                TemperatureSensorSP.s = IPS_IDLE;
                DEBUG(INDI::Logger::DBG_SESSION, "Temperature is sourced from sensor connected to the FPGA analogue pin 14");
            }
            if (TemperatureSensorS[2].s == ISS_ON)
            {
                IERmTimer(temperatureCompensationID);
                TemperatureSensorSP.s = IPS_IDLE;
                DEBUG(INDI::Logger::DBG_SESSION, "Temperature is sourced from sensor connected to the FPGA analogue pin 15");
            }

            IDSetSwitch(&TemperatureSensorSP, nullptr);
            return true;
        }
        // handle temperature compensation
        if (!strcmp(name, TemperatureCompensateSP.name))
        {
            IUUpdateSwitch(&TemperatureCompensateSP, states, names, n);

            if (TemperatureCompensateS[0].s == ISS_ON)
            {
                if (!temperatureCompensationID)
                    temperatureCompensationID =
                        IEAddTimer(TEMPERATURE_COMPENSATION_TIMEOUT, temperatureCompensationHelper, this);
                TemperatureCompensateSP.s = IPS_OK;
                DEBUG(INDI::Logger::DBG_SESSION, "Temperature compensation ENABLED.");
            }

            if (TemperatureCompensateS[1].s == ISS_ON)
            {
                IERmTimer(temperatureCompensationID);
                TemperatureCompensateSP.s = IPS_IDLE;
                DEBUG(INDI::Logger::DBG_SESSION, "Temperature compensation DISABLED.");
            }

            IDSetSwitch(&TemperatureCompensateSP, nullptr);
            return true;
        }
        // handle temperature compensation
        if (!strcmp(name, MotorTypeSP.name))
        {
            IUUpdateSwitch(&MotorTypeSP, states, names, n);

            if (MotorTypeS[0].s == ISS_ON)
            {
	              koheron_interface->SetFocuserMotorType(false);
                MotorTypeSP.s = IPS_OK;
                DEBUG(INDI::Logger::DBG_SESSION, "Enable DRV8825.");
            }

            if (MotorTypeS[1].s == ISS_ON)
            {
	              koheron_interface->SetFocuserMotorType(true);
                MotorTypeSP.s = IPS_IDLE;
                DEBUG(INDI::Logger::DBG_SESSION, "Enable TMC2226.");
            }

            IDSetSwitch(&MotorTypeSP, nullptr);
            return true;
        }
        // handle temperature compensation
        if (!strcmp(name, ServerDebugSP.name))
        {
            IUUpdateSwitch(&ServerDebugSP, states, names, n);

            if (ServerDebugrS[0].s == ISS_ON)
            {
	              koheron_interface->set_debug(true);
                ServerDebugSP.s = IPS_OK;
                DEBUG(INDI::Logger::DBG_SESSION, "Enable Koheron server logging.");
            }

            if (ServerDebugrS[1].s == ISS_ON)
            {
	              koheron_interface->set_debug(false);
                ServerDebugSP.s = IPS_IDLE;
                DEBUG(INDI::Logger::DBG_SESSION, "Disable Koheron server logging.");
            }

            IDSetSwitch(&ServerDebugSP, nullptr);
            return true;
        }
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool FpgaFocuser::ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	// first we check if it's for our device
	if (!strcmp(dev, getDeviceName()))
	{
		// handle active devices
		if (!strcmp(name, ActiveTelescopeTP.name))
		{
				IUUpdateText(&ActiveTelescopeTP,texts,names,n);

				IUFillNumberVector(&ScopeParametersNP, ScopeParametersN, 2, ActiveTelescopeT[0].text, "TELESCOPE_INFO", "Scope Properties", OPTIONS_TAB, IP_RW, 60, IPS_OK);
				IDSnoopDevice(ActiveTelescopeT[0].text, "TELESCOPE_INFO");

				ActiveTelescopeTP.s=IPS_OK;
				IDSetText(&ActiveTelescopeTP, nullptr);
				DEBUGF(INDI::Logger::DBG_SESSION, "Active telescope set to %s.", ActiveTelescopeT[0].text);
				return true;
		}
	}

	return INDI::Focuser::ISNewText(dev,name,texts,names,n);
}

bool FpgaFocuser::ISSnoopDevice (XMLEle *root)
{
	if (IUSnoopNumber(root, &ScopeParametersNP) == 0)
	{
		getFocuserInfo();
		DEBUGF(INDI::Logger::DBG_DEBUG, "Telescope parameters: %0.0f, %0.0f.", ScopeParametersN[0].value, ScopeParametersN[1].value);
		return true;
	}

	return INDI::Focuser::ISSnoopDevice(root);
}

bool FpgaFocuser::saveConfigItems(FILE *fp)
{


	DEBUG(INDI::Logger::DBG_DEBUG, "saving ...");
  tcpConnection->saveConfigItems(fp);
	IUSaveConfigSwitch(fp, &FocusReverseSP);
	IUSaveConfigSwitch(fp, &TemperatureSensorSP);
	IUSaveConfigSwitch(fp, &ServerDebugSP);
	IUSaveConfigSwitch(fp, &MotorTypeSP);
	IUSaveConfigSwitch(fp, &TemperatureCompensateSP);
	IUSaveConfigNumber(fp, &FocusMaxPosNP);
	//IUSaveConfigNumber(fp, &FocusAbsPosNP);
	IUSaveConfigNumber(fp, &FocusBacklashPeriodNP);
	IUSaveConfigNumber(fp, &FocuserTravelNP);
	IUSaveConfigNumber(fp, &PresetNP);
	IUSaveConfigNumber(fp, &TemperatureCoefNP);
	IUSaveConfigText(fp, &ActiveTelescopeTP);
  IUSaveConfigSwitch(fp, &FocusBacklashSP);
  IUSaveConfigNumber(fp, &FocusBacklashNP);
  IUSaveConfigNumber(fp, &FocusSpeedNP);
	return true;
}

void FpgaFocuser::TimerHit()
{
}

bool FpgaFocuser::SyncFocuser( uint32_t ticks )
{
  if ( !isConnected() ) return false;

	koheron_interface->SetFocuserPosition(ticks);
  updateStatusFunc();
	DEBUGF(INDI::Logger::DBG_SESSION, "Focuser synced to %d.", ticks);
  return true;
}

bool FpgaFocuser::SetFocuserBacklash(int32_t steps)
{
  if ( !isConnected() ) return false;
	if (!koheron_interface->set_backlash_cycles(steps)){
	  DEBUG(INDI::Logger::DBG_WARNING, "Failed to set backlash steps.");
    return false;
  }
	if (IUFindOnSwitchIndex(&FocusBacklashSP) == DefaultDevice::INDI_ENABLED) koheron_interface->enable_backlash(true);
  DEBUGF(INDI::Logger::DBG_SESSION, "Backlash set to %0.0f steps", FocusBacklashN[0].value);
	return true;
}

bool FpgaFocuser::SetFocuserBacklashEnabled(bool enabled)
{
  if ( !isConnected() ) return false;
	if (!koheron_interface->enable_backlash(enabled)) {
	  DEBUG(INDI::Logger::DBG_WARNING, "Failed to toggle backlash.");
    return false;
  }
	DEBUG(INDI::Logger::DBG_SESSION, "Focuser backlash toggled.");
	return true;
}

bool FpgaFocuser::SetFocuserSpeed(int speed)
{
  if ( !isConnected() ) return false;
	DEBUGF(INDI::Logger::DBG_SESSION, "Focuser speed set to: %d us", speed);
	return true;
}

bool FpgaFocuser::AbortFocuser()
{
  if ( !isConnected() ) return false;
	koheron_interface->StopFocuser(false);
	DEBUG(INDI::Logger::DBG_SESSION, "Focuser motion aborted.");
  updateStatusFunc();
	return true;
}

IPState FpgaFocuser::MoveRelFocuser(FocusDirection dir, int ticks)
{
  updateStatusFunc();
	if (hw_is_running)
	{
		DEBUG(INDI::Logger::DBG_WARNING, "Focusser still running from last time");
		return IPS_ALERT;
	}
  if (!koheron_interface->FocuserIncrement(ticks, FocusSpeedN[0].value/motorPeriodUs, dir))
  {
	  DEBUGF(INDI::Logger::DBG_SESSION, "%s: Failed to start focuser.", __func__);
		return IPS_ALERT;
  }
  updateStatusFunc();
	lastTemperature = FocusTemperatureN[0].value; // register last temperature
	DEBUGF(INDI::Logger::DBG_SESSION, "%s: Focuser motion started.", __func__);
  FocusRelPosNP.s = IPS_BUSY;
  IDSetNumber(&FocusRelPosNP, nullptr);

  return IPS_OK;
}

IPState FpgaFocuser::MoveAbsFocuser(int targetTicks)
{
  updateStatusFunc();
	if (hw_is_running)
	{
		DEBUG(INDI::Logger::DBG_WARNING, "Focusser still running from last time");
		return IPS_ALERT;
	}
	if (targetTicks < FocusAbsPosN[0].min || targetTicks > FocusAbsPosN[0].max)
	{
		DEBUG(INDI::Logger::DBG_WARNING, "Requested position is out of range.");
		return IPS_ALERT;
	}

  int tmp  = FocusAbsPosN[0].value;
	if (targetTicks == tmp)
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Already at the requested position.");
		return IPS_OK;
	}

  bool dir = tmp < targetTicks;
  if (!koheron_interface->FocuserGotoTarget(targetTicks, FocusSpeedN[0].value/motorPeriodUs, dir))
  {
		DEBUGF(INDI::Logger::DBG_WARNING, "%s: Failed to start motion", __func__);
		return IPS_ALERT;
  }
	// update abspos value and status
	DEBUGF(INDI::Logger::DBG_SESSION, "%s: Focuser motion started, current: %0.0f, target: %d, direction: %s.", __func__, FocusAbsPosN[0].value, targetTicks, dir ? "true" : "false");
  updateStatusFunc();


	// reset last temperature
	lastTemperature = FocusTemperatureN[0].value; // register last temperature
  FocusAbsPosNP.s = IPS_BUSY;
  IDSetNumber(&FocusAbsPosNP, nullptr);

	return IPS_OK;
}

bool FpgaFocuser::ReverseFocuser(bool enabled)
{
  reverse_direction = enabled;
	if (enabled)
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Reverse direction ENABLED.");
	} else {
		DEBUG(INDI::Logger::DBG_SESSION, "Reverse direction DISABLED.");
	}
	return true;
}

int FpgaFocuser::savePosition(int pos)
{
	FILE * pFile;
	char posFileName[MAXRBUF];
	char buf [100];

	if (getenv("INDICONFIG"))
	{
		snprintf(posFileName, MAXRBUF, "%s.position", getenv("INDICONFIG"));
	} else {
		snprintf(posFileName, MAXRBUF, "%s/.indi/%s.position", getenv("HOME"), getDeviceName());
	}


	if (pos == -1)
	{
		pFile = fopen (posFileName,"r");
		if (pFile == NULL)
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "Failed to open file %s.", posFileName);
			return -1;
		}

		fgets (buf , 100, pFile);
		pos = atoi (buf);
		DEBUGF(INDI::Logger::DBG_DEBUG, "Reading position %d from %s.", pos, posFileName);
	} else {
		pFile = fopen (posFileName,"w");
		if (pFile == NULL)
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "Failed to open file %s.", posFileName);
			return -1;
		}

		sprintf(buf, "%d", pos);
		fputs (buf, pFile);
		DEBUGF(INDI::Logger::DBG_DEBUG, "Writing position %s to %s.", buf, posFileName);
	}

	fclose (pFile);

	return pos;
}

bool FpgaFocuser::readtemp()
{

	// set busy
	FocusTemperatureNP.s=IPS_BUSY;
	IDSetNumber(&FocusTemperatureNP, nullptr);


	float val = 0;
  
  if (TemperatureSensorS[0].s == ISS_ON)
    val = (float)koheron_interface->GetTemp_pi1w();
  else if (TemperatureSensorS[1].s == ISS_ON)
    val = (float)koheron_interface->GetTemp_fpga(20);
  else
    val = (float)koheron_interface->GetTemp_fpga(28);

	FocusTemperatureNP.s=IPS_BUSY;
	IDSetNumber(&FocusTemperatureNP, nullptr);

  if (val > 100 || val < -99)
  {
	  DEBUGF(INDI::Logger::DBG_ERROR, "Invalid value received %0.0f", val);
    return false;
  }
	FocusTemperatureN[0].value = (float) val;

	// set OK
	FocusTemperatureNP.s=IPS_OK;
	IDSetNumber(&FocusTemperatureNP, nullptr);
	DEBUGF(INDI::Logger::DBG_DEBUG, "Temperature: %.2f°C", val);

	return true;
}

void FpgaFocuser::getFocuserInfo()
{
	// https://www.innovationsforesight.com/education/how-much-focus-error-is-too-much/
	float travel_mm = (float) FocuserTravelN[0].value;
	float aperture = (float) ScopeParametersN[0].value;
	float focal = (float) ScopeParametersN[1].value;
	float f_ratio;

	// handle no snooping data from telescope
	if ( aperture * focal != 0 )
	{
		f_ratio = focal / aperture;
	} else {
		f_ratio =  0;
		DEBUG(INDI::Logger::DBG_DEBUG, "No telescope focal length and/or aperture info available.");
	}

	float cfz = 4.88 * 0.520 * pow(f_ratio, 2); // CFZ = 4.88 · λ · f^2
	float step_size = 1000.0 * travel_mm / FocusMaxPosN[0].value;
	float steps_per_cfz = (int) cfz / step_size;

	if ( steps_per_cfz >= 4  )
	{
		FocuserInfoNP.s = IPS_OK;
	}
	else if ( steps_per_cfz > 2 && steps_per_cfz < 4 )
	{
		FocuserInfoNP.s = IPS_BUSY;
	} else {
		FocuserInfoNP.s = IPS_ALERT;
	}

	FocuserInfoN[0].value = step_size;
	FocuserInfoN[1].value = cfz;
	FocuserInfoN[2].value = steps_per_cfz;
	IDSetNumber(&FocuserInfoNP, nullptr);

	DEBUGF(INDI::Logger::DBG_DEBUG, "Focuser Info: %0.2f %0.2f %0.2f.", FocuserInfoN[0].value, FocuserInfoN[1].value, FocuserInfoN[2].value);
}


void FpgaFocuser::updateStatusHelper(void *context)
{
	static_cast<FpgaFocuser*>(context)->updateStatus();
}

void FpgaFocuser::updateTemperatureHelper(void *context)
{
	static_cast<FpgaFocuser*>(context)->updateTemperature();
}

void FpgaFocuser::temperatureCompensationHelper(void *context)
{
	static_cast<FpgaFocuser*>(context)->temperatureCompensation();
}

void FpgaFocuser::updateStatusFunc()
{
  std::array<bool, 8> response = koheron_interface->GetFocuserAxisStatus();
  hw_is_initialized      = response[0];
  hw_is_running          = response[1];
  hw_direction = response[2];
  static bool detected_motion	= false;
	FocusAbsPosN[0].value = koheron_interface->GetFocuserPosition();
  if (!hw_is_running)
  {
	    FocusRelPosNP.s = IPS_OK;
	    IDSetNumber(&FocusRelPosNP, nullptr);
	    FocusAbsPosNP.s = IPS_OK;
			if (detected_motion) 
      {
        DEBUG(INDI::Logger::DBG_SESSION, "Hardware motion stopped.");
	      savePosition((int) FocusAbsPosN[0].value); 
        detected_motion	= false;
      }
  }
  else
  {
    detected_motion	= true;
	  FocusAbsPosNP.s = IPS_BUSY;
	  updateStatusID = IEAddTimer(STATUS_UPDATE_TIMEOUT, updateStatusHelper, this);
  }
  IDSetNumber(&FocusAbsPosNP, nullptr);
}
void FpgaFocuser::updateStatus()
{
	if (isConnected())
  {
    updateStatusFunc ();
  }
}
void FpgaFocuser::updateTemperature()
{
	if (!updateTemperatureID) updateTemperatureID = IEAddTimer(TEMPERATURE_UPDATE_TIMEOUT, updateTemperatureHelper, this); // set temperature update timer
	if (isConnected())
	  readtemp();
	updateTemperatureID = IEAddTimer(TEMPERATURE_UPDATE_TIMEOUT, updateTemperatureHelper, this); // set temperature update timer
}

void FpgaFocuser::temperatureCompensation()
{
	if (!isConnected())
		return;

	if ( TemperatureCompensateS[0].s == ISS_ON && FocusTemperatureN[0].value != lastTemperature )
	{
		float deltaTemperature = FocusTemperatureN[0].value - lastTemperature; // change of temperature from last focuser movement
		float thermalExpansionRatio = TemperatureCoefN[0].value * ScopeParametersN[1].value / 1000; // termal expansion in micrometers per 1 celcius degree
		float thermalExpansion = thermalExpansionRatio * deltaTemperature; // actual thermal expansion

		DEBUGF(INDI::Logger::DBG_DEBUG, "Thermal expansion of %0.1f μm due to temperature change of %0.2f°C", thermalExpansion, deltaTemperature);

		if ( abs(thermalExpansion) > FocuserInfoN[1].value / 2)
		{
			int thermalAdjustment = round((thermalExpansion / FocuserInfoN[0].value) / 2); // adjust focuser by half number of steps to keep it in the center of cfz
			MoveAbsFocuser(FocusAbsPosN[0].value + thermalAdjustment); // adjust focuser position
			lastTemperature = FocusTemperatureN[0].value; // register last temperature
			DEBUGF(INDI::Logger::DBG_SESSION, "Focuser adjusted by %d steps due to temperature change by %0.2f°C", thermalAdjustment, deltaTemperature);
		}
	}

	temperatureCompensationID = IEAddTimer(TEMPERATURE_COMPENSATION_TIMEOUT, temperatureCompensationHelper, this);
}
