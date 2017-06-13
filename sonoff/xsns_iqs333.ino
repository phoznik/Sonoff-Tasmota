/*
  xsns_iqs333.ino - IQS333 Capacitive touch controller support for Sonoff-Tasmota
  Copyright (C) 2017  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_I2C
#ifdef USE_IQS
/*********************************************************************************************\
 * IQS333 - Azoteq ProxSense capacitive proximity and touch controller
 *
 * Source: Daryl Sielaff
\*********************************************************************************************/

#define IQS333_ADDR        0x64

#define IQS333_PROD_NUM    0x54
#define IQS333_VERSION_NUM 0x02

// Communication Command / Address Structure on IQS333 - ie. Memory Map
#define	IQS_REG_VERSION_INFO		0x00	// Product number can be read      : 2 bytes
#define IQS_REG_FLAGS 			0x01	// System flags and events         : 1 byte
#define IQS_REG_WHEEL_COORDS		0x02	// Wheel coordinates - 2 wheels    : 4 bytes
#define IQS_REG_TOUCH_BYTES     	0x03	// Touch channels                  : 2 bytes
#define IQS_REG_COUNTS                	0x04	// Count Values                    :20 bytes
#define IQS_REG_LTA              	0x05	// LTA Values                      :20 bytes
#define IQS_REG_MULTIPLIERS            	0x06	// Multipliers Values              :10 bytes
#define IQS_REG_COMPENSATION            0x07	// Compensation Values (PCC)       :10 bytes
#define IQS_REG_PROXSETTINGS           	0x08	// Prox Settings - Various         : 6 bytes
#define IQS_REG_THRESHOLDS              0x09	// Threshold Values                :10 bytes
#define IQS_REG_TIMINGS                 0x0A    // Timings                         : 5 bytes
#define	IQS_REG_ATI_TARGETS		0x0B	// Targets for ATI                 : 2 bytes
#define IQS_REG_PWM 			0x0C	// PWM Settings                    : 4 bytes
#define IQS_REG_PWM_LIMIT		0x0D	// PWM Limits and speed            : 2 bytes
#define IQS_REG_ACTIVE_CHANNELS     	0x0E	// Active channels                 : 2 bytes
#define IQS_REG_BUZZER           	0x0F	// Buzzer Output                   : 1 byte

uint8_t iqsaddr;
uint8_t iqstype;
char iqstype_s[7];

void iqs_rdy_interrupt()  ICACHE_RAM_ATTR;

void iqs_rdy_interrupt()
{
  //read event here
}

boolean iqs_init()
{
  pinMode(pin[], INPUT_PULLUP); //Configure the RDY pin (In event mode the IQS will pull RDY low when it has an event to read
  attachInterrupt(pin[GPIO_I2C_RDY], iqs_rdy_interrupt, FALLING);

  //Do initialization things here to configure sliders
  //Enable event mode on the IQS333
  
  return true;
}

uint8_t iqs_detect()
{
  if (iqstype) {
    return true;
  }

  char log[LOGSZ];
  boolean success = false;

  iqsaddr = IQS333_ADDR;
  iqstype = i2c_read8(iqsaddr, IQS_REG_VERSION_INFO);
  success = iqs_init();
  switch (iqstype) {
  case IQS333_PROD_NUM:
    strcpy_P(iqstype_s, PSTR("IQS333"));
    break;
  default:
    strcpy_P(iqstype_s, PSTR("IQS?"));
  }
  if (success) {
    snprintf_P(log, sizeof(log), PSTR("I2C: %s found at address 0x%x"), iqstype_s, iqsaddr);
    addLog(LOG_LEVEL_DEBUG, log);
  } else {
    iqstype = 0;
  }
  return success;
}

/*********************************************************************************************\
 * Presentation
\*********************************************************************************************/

void iqs_mqttPresent(char* svalue, uint16_t ssvalue, uint8_t* djson)
{
  if (!iqstype) {
    return;
  }

  char stemp1[10];
  char stemp2[10];

  float t = 40; //htu21_readTemperature();
  float h = 50; //htu21_readHumidity();
  h = 55; //htu21_compensatedHumidity(h, t);
  dtostrf(t, 1, sysCfg.flag.temperature_resolution, stemp1);
  dtostrf(h, 1, sysCfg.flag.humidity_resolution, stemp2);
  snprintf_P(svalue, ssvalue, PSTR("%s, \"IQS333\":{\"Temperature\":%s, \"Pressure\":%s}"), svalue, stemp1, stemp2);  //dummy data for now
  *djson = 1;
#ifdef USE_DOMOTICZ
  domoticz_sensor2(stemp1, stemp2);
#endif  // USE_DOMOTICZ
}

#ifdef USE_WEBSERVER
String iqs_webPresent()
{
  String page = "";
  if (iqstype) {
    //create page here
  }
  return page;
}
#endif  // USE_WEBSERVER
#endif  // USE_IQS
#endif  // USE_I2C
