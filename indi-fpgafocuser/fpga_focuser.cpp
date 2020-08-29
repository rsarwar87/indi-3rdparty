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
 * - Save position in xml instead flat file
 * - Handle AbortFocuser()
 * - Add temperature compensation auto learning and save temperature compensation curve to xml
 * - Add simulation mode
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
	FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_REVERSE | FOCUSER_CAN_ABORT | FOCUSER_CAN_SYNC);
	Focuser::setSupportedConnections(CONNECTION_NONE);
}

FpgaFocuser::~FpgaFocuser()
{
	// delete properties independent of connection status
  koheron_interface.reset();
}

bool FpgaFocuser::Connect()
{
  const char * ip = IPAddress[0].text;
  uint32_t port = 36000; 
  if (!validateIpAddress(IPAddress[0].text))
  {
			DEBUGF(INDI::Logger::DBG_ERROR, "Invalid IP address: %si:%d", ip, port);
			return false;
  }

  try {
      koheron_interface = std::make_unique<indi_focuser_interface>(ip, port);
  }
  catch (const std::exception& e)
  {
			DEBUGF(INDI::Logger::DBG_ERROR, "Could not connect to server: %s: %s"
          , IPAddress[0].text, e.what());
			return false;
  }
  updateStatusFunc();

  if (!hw_is_initialized) 
    koheron_interface->Initialize();
	motorPeriodUs = 1000000.0 / koheron_interface->GetTimerInterruptFreq();
	FocusStepPeriod[0].min = koheron_interface->get_minimum_period()*3*motorPeriodUs;
	FocusStepPeriod[0].max = koheron_interface->get_maximum_period()*motorPeriodUs;
	FocusBacklashN[1].min = FocusStepPeriod[0].min;
	FocusBacklashN[1].max = FocusStepPeriod[0].max;
	FocusMaxPosN[0].max = koheron_interface->GetGridPerRevolution();
	FocusMaxPosN[0].min = FocusMaxPosN[0].max;
	FocusMaxPosN[0].value = FocusMaxPosN[0].max;

	FocusRelPosN[0].max = koheron_interface->GetGridPerRevolution()/100;

	FocusAbsPosN[0].min = FocusSyncN[0].min = 0;
	FocusAbsPosN[0].max = FocusSyncN[0].max = FocusMaxPosN[0].value;
	FocusAbsPosN[0].step = (int) FocusAbsPosN[0].max / 1000;

  if (savePosition(-1) != -1)
  {
	  FocusAbsPosN[0].value = (int) savePosition(-1);
    koheron_interface->SetFocuserPosition(FocusAbsPosN[0].value);
  }
  else
  {
	  FocusAbsPosN[0].value = koheron_interface->GetFocuserPosition();
  }

	updateStatusID = IEAddTimer(STATUS_UPDATE_TIMEOUT, updateStatusHelper, this);
	IPAddressP.s=IPS_BUSY;
	IDSetText(&IPAddressP, nullptr);
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
	IPAddressP.s=IPS_IDLE;
	IDSetText(&IPAddressP, nullptr);

	// Stop timers
	IERmTimer(updateTemperatureID);
	IERmTimer(temperatureCompensationID);

	DEBUG(INDI::Logger::DBG_SESSION, "Astroberry Focuser disconnected successfully.");

	return true;
}

bool FpgaFocuser::initProperties()
{
	INDI::Focuser::initProperties();

	// Focuser Info
	IUFillNumber(&FocuserInfoN[0], "CFZ_STEP_ACT", "Step Size (μm)", "%0.2f", 0, 1000, 1, 0);
	IUFillNumber(&FocuserInfoN[1], "CFZ", "Critical Focus Zone (μm)", "%0.2f", 0, 1000, 1, 0);
	IUFillNumber(&FocuserInfoN[2], "STEPS_PER_CFZ", "Steps / Critical Focus Zone", "%0.0f", 0, 1000, 1, 0);
	IUFillNumberVector(&FocuserInfoNP, FocuserInfoN, 3, getDeviceName(), "FOCUSER_PARAMETERS", "Focuser Info", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);


	// Step delay setting
	IUFillNumber(&FocusStepPeriod[0], "FOCUS_STEPDELAY_VALUE", "microseconds", "%0.0f", 30, 19000, 1, 45);
	IUFillNumberVector(&FocusStepPeriodP, FocusStepPeriod, 1, getDeviceName(), "FOCUS_STEPDELAY", "Step Delay", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Backlash setting
	IUFillNumber(&FocusBacklashN[0], "FOCUS_BACKLASH_VALUE", "steps", "%0.0f", 0, 10000, 1, 150);
	IUFillNumber(&FocusBacklashN[1], "FOCUS_BACKLASH_PERIOD_VALUE", "microseconds", "%0.0f", 15, 19000, 1, 45);
	IUFillNumberVector(&FocusBacklashNP, FocusBacklashN, 2, getDeviceName(), "FOCUS_BACKLASH", "Backlash", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Reset absolute possition
	IUFillSwitch(&ResetAbsPosS[0],"RESET_ABS","Purge",ISS_OFF);
	IUFillSwitchVector(&ResetAbsPosSP,ResetAbsPosS,1,getDeviceName(),"RESET_ABS_SW","Saved Position",OPTIONS_TAB,IP_RW,ISR_1OFMANY,0,IPS_IDLE);

	// Koheron IP address
	IUFillText(&IPAddress[0], "KOHERON_SERVER_ADDRESS", "Sever Address", "127.0.0.1");
	IUFillTextVector(&IPAddressP, IPAddress, 1, getDeviceName(), "KOHERON_SERVER_ADDRESS", "Server Address", CONNECTION_TAB, IP_RW, 0, IPS_IDLE);
  registerProperty(&IPAddressP, INDI_TEXT);

	// Active telescope setting
	IUFillText(&ActiveTelescopeT[0], "ACTIVE_TELESCOPE_NAME", "Telescope", "Telescope Simulator");
	IUFillTextVector(&ActiveTelescopeTP, ActiveTelescopeT, 1, getDeviceName(), "ACTIVE_TELESCOPE", "Snoop devices", OPTIONS_TAB,IP_RW, 0, IPS_IDLE);

	// Maximum focuser travel
	IUFillNumber(&FocuserTravelN[0], "FOCUSER_TRAVEL_VALUE", "mm", "%0.0f", 10, 200, 10, 10);
	IUFillNumberVector(&FocuserTravelNP, FocuserTravelN, 1, getDeviceName(), "FOCUSER_TRAVEL", "Max Travel", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Focuser temperature
	IUFillNumber(&FocusTemperatureN[0], "FOCUS_TEMPERATURE_VALUE", "°C", "%0.2f", -50, 50, 1, 0);
	IUFillNumberVector(&FocusTemperatureNP, FocusTemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

	// Compensate for backlash
	IUFillSwitch(&BacklashCorrectionS[0], "Enable", "", ISS_OFF);
	IUFillSwitch(&BacklashCorrectionS[1], "Disable", "", ISS_ON);
	IUFillSwitchVector(&BacklashCorrectionSP, BacklashCorrectionS, 2, getDeviceName(), "BacklashCompensate", "Backlash Compensate", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Temperature Coefficient
	IUFillNumber(&TemperatureCoefN[0], "μm/m°C", "", "%.1f", 0, 50, 1, 0);
	IUFillNumberVector(&TemperatureCoefNP, TemperatureCoefN, 1, getDeviceName(), "Temperature Coefficient", "", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

	// Compensate for temperature
	IUFillSwitch(&TemperatureCompensateS[0], "Enable", "", ISS_OFF);
	IUFillSwitch(&TemperatureCompensateS[1], "Disable", "", ISS_ON);
	IUFillSwitchVector(&TemperatureCompensateSP, TemperatureCompensateS, 2, getDeviceName(), "Temperature Compensate", "", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Snooping params
	IUFillNumber(&ScopeParametersN[0], "TELESCOPE_APERTURE", "Aperture (mm)", "%g", 10, 5000, 0, 0.0);
	IUFillNumber(&ScopeParametersN[1], "TELESCOPE_FOCAL_LENGTH", "Focal Length (mm)", "%g", 10, 10000, 0, 0.0);
	IUFillNumberVector(&ScopeParametersNP, ScopeParametersN, 2, ActiveTelescopeT[0].text, "TELESCOPE_INFO", "Scope Properties", OPTIONS_TAB, IP_RW, 60, IPS_OK);

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
		defineText(&IPAddressP);
		defineText(&ActiveTelescopeTP);
		defineNumber(&FocuserTravelNP);
		defineSwitch(&FocusMotionSP);
		defineNumber(&FocuserInfoNP);
		defineNumber(&FocusStepPeriodP);
		defineNumber(&FocusBacklashNP);
		defineSwitch(&ResetAbsPosSP);

		IDSnoopDevice(ActiveTelescopeT[0].text, "TELESCOPE_INFO");

		if (readtemp())
		{
			defineNumber(&FocusTemperatureNP);
			defineNumber(&TemperatureCoefNP);
			defineSwitch(&TemperatureCompensateSP);
			defineSwitch(&BacklashCorrectionSP);
			readtemp(); // update immediately
			lastTemperature = FocusTemperatureN[0].value; // init last temperature
			IERmTimer(updateTemperatureID);
			updateTemperatureID = IEAddTimer(TEMPERATURE_UPDATE_TIMEOUT, updateTemperatureHelper, this); // set temperature update timer
			IERmTimer(temperatureCompensationID);
			temperatureCompensationID = IEAddTimer(TEMPERATURE_COMPENSATION_TIMEOUT, temperatureCompensationHelper, this); // set temperature compensation timer
		}

	} else {
		deleteProperty(IPAddressP.name);
		deleteProperty(ActiveTelescopeTP.name);
		deleteProperty(FocuserTravelNP.name);
		deleteProperty(FocusMotionSP.name);
		deleteProperty(FocuserInfoNP.name);
		deleteProperty(FocusStepPeriodP.name);
		deleteProperty(FocusBacklashNP.name);
		deleteProperty(ResetAbsPosSP.name);
		deleteProperty(FocusTemperatureNP.name);
		deleteProperty(TemperatureCoefNP.name);
		deleteProperty(TemperatureCompensateSP.name);
		deleteProperty(BacklashCorrectionSP.name);
	}

	return true;
}

bool FpgaFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    // first we check if it's for our device
    if (!strcmp(dev, getDeviceName()))
    {
        // handle focus maximum position
        if (!strcmp(name, FocusMaxPosNP.name))
        {
            IUUpdateNumber(&FocusMaxPosNP, values, names, n);

            FocusAbsPosN[0].max = FocusMaxPosN[0].value;
            IUUpdateMinMax(&FocusAbsPosNP); // This call is not INDI protocol compliant

            FocusAbsPosNP.s = IPS_OK;
            IDSetNumber(&FocusMaxPosNP, nullptr);
            getFocuserInfo();
            return true;
        }

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
        if (!strcmp(name, FocusBacklashNP.name))
        {
            IUUpdateNumber(&FocusBacklashNP, values, names, n);
            FocusBacklashNP.s = IPS_BUSY;
            IDSetNumber(&FocusBacklashNP, nullptr);
            FocusBacklashNP.s = IPS_OK;
            IDSetNumber(&FocusBacklashNP, nullptr);
            DEBUGF(INDI::Logger::DBG_SESSION, "Backlash set to %0.0f steps and a period of %0.0f us.", FocusBacklashN[0].value, FocusBacklashN[1].value);
            return true;
        }

        // handle focus step delay
        if (!strcmp(name, FocusStepPeriodP.name))
        {
            IUUpdateNumber(&FocusStepPeriodP, values, names, n);
            FocusStepPeriodP.s = IPS_BUSY;
            IDSetNumber(&FocusStepPeriodP, nullptr);
            FocusStepPeriodP.s = IPS_OK;
            IDSetNumber(&FocusStepPeriodP, nullptr);
            DEBUGF(INDI::Logger::DBG_SESSION, "Step delay set to %0.0f us.", FocusStepPeriod[0].value);
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
            FocusAbsPosN[0].value = koheron_interface->GetFocuserHomePosition();
            IDSetNumber(&FocusAbsPosNP, nullptr);
            savePosition(0);

            DEBUG(INDI::Logger::DBG_SESSION, "Absolute Position reset to 0.");

            ResetAbsPosSP.s = IPS_IDLE;
            IDSetSwitch(&ResetAbsPosSP, nullptr);
            return true;
        }

        // handle backlash compensation
        if (!strcmp(name, BacklashCorrectionSP.name))
        {
            IUUpdateSwitch(&BacklashCorrectionSP, states, names, n);

            if (BacklashCorrectionS[0].s == ISS_ON)
            {
                if (!isConnected()){
                  DEBUG(INDI::Logger::DBG_WARNING, "Device not connected to.");
                  return false;
                }
                FocusBacklashNP.s = IPS_BUSY;
                koheron_interface->set_backlash_period(FocusBacklashN[1].value/motorPeriodUs);
                koheron_interface->set_backlash_cycles(FocusBacklashN[0].value);
                koheron_interface->enable_backlash(true);
                BacklashCorrectionSP.s = IPS_OK;
                DEBUG(INDI::Logger::DBG_SESSION, "Hardware backlash compensation ENABLED.");
            }

            if (BacklashCorrectionS[1].s == ISS_ON)
            {
                if (!isConnected()){
                  DEBUG(INDI::Logger::DBG_WARNING, "Device not connected to.");
                  return false;
                }
                koheron_interface->enable_backlash(false);
                FocusBacklashNP.s = IPS_IDLE;
                BacklashCorrectionSP.s = IPS_IDLE;
                DEBUG(INDI::Logger::DBG_SESSION, "Hardware backlash compensation DISABLED.");
            }

            IDSetSwitch(&BacklashCorrectionSP, nullptr);
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
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool FpgaFocuser::ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	// first we check if it's for our device
	if (!strcmp(dev, getDeviceName()))
	{
		// handle active devices
		if (!strcmp(name, IPAddressP.name))
		{
				IUUpdateText(&IPAddressP,texts,names,n);
				IPAddressP.s=IPS_BUSY;
				IDSetText(&IPAddressP, nullptr);
				IPAddressP.s=IPS_OK;
				IDSetText(&IPAddressP, nullptr);
				DEBUGF(INDI::Logger::DBG_SESSION, "IP Address set to %s.", IPAddress[0].text);
				return true;
		}
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
	IUSaveConfigSwitch(fp, &FocusReverseSP);
	IUSaveConfigSwitch(fp, &TemperatureCompensateSP);
	IUSaveConfigSwitch(fp, &BacklashCorrectionSP);
	IUSaveConfigNumber(fp, &FocusMaxPosNP);
	IUSaveConfigNumber(fp, &FocusStepPeriodP);
	IUSaveConfigNumber(fp, &FocusBacklashNP);
	IUSaveConfigNumber(fp, &FocuserTravelNP);
	IUSaveConfigNumber(fp, &PresetNP);
	IUSaveConfigNumber(fp, &TemperatureCoefNP);
	IUSaveConfigText(fp, &IPAddressP);
	IUSaveConfigText(fp, &ActiveTelescopeTP);
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
  if (!koheron_interface->FocuserIncrement(ticks, FocusStepPeriod[0].value/motorPeriodUs, dir))
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

	if (targetTicks == FocusAbsPosN[0].value)
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Already at the requested position.");
		return IPS_OK;
	}

  bool dir = FocusAbsPosN[0].value < targetTicks;
  if (!koheron_interface->FocuserGotoTarget(targetTicks, FocusStepPeriod[0].value/motorPeriodUs, reverse_direction ? !dir : dir))
  {
		DEBUGF(INDI::Logger::DBG_WARNING, "%s: Failed to start motion", __func__);
		return IPS_ALERT;
  }
  updateStatusFunc();
	// update abspos value and status
	DEBUGF(INDI::Logger::DBG_SESSION, "%s: Focuser motion started.", __func__);


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


	float val = (float) koheron_interface->GetTemp_pi1w();

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
  detected_motion	= false;
	FocusAbsPosN[0].value = koheron_interface->GetFocuserPosition();
    if (!hw_is_running)
    {
	    FocusRelPosNP.s = IPS_OK;
	    IDSetNumber(&FocusRelPosNP, nullptr);
	    FocusAbsPosNP.s = IPS_OK;
	    savePosition((int) FocusAbsPosN[0].value); 
			if (detected_motion) DEBUG(INDI::Logger::DBG_SESSION, "Hardware motion stopped.");
      detected_motion	= false;
    }
    else
    {
      detected_motion	= true;
	    FocusAbsPosNP.s = IPS_BUSY;

    }
  IDSetNumber(&FocusAbsPosNP, nullptr);
}
void FpgaFocuser::updateStatus()
{
	if (isConnected())
  {
    updateStatusFunc ();
  }
	updateStatusID = IEAddTimer(STATUS_UPDATE_TIMEOUT, updateStatusHelper, this);
}
void FpgaFocuser::updateTemperature()
{
	if (isConnected())
	  readtemp();

	updateTemperatureID = IEAddTimer(TEMPERATURE_UPDATE_TIMEOUT, updateTemperatureHelper, this);
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
