/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
    Please note the long strings of data sent mean the *RTS* pin is
    required with UART to slow down data sent to the Bluefruit LE!  
*/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLEGatt.h"
#include "IEEE11073float.h"

#include "BluefruitConfig.h"

// Pin definitions
#define SUR_TEMP_PIN      A0          // Analog input pin connect to temperature sensor SUR pin
#define OBJ_TEMP_PIN      A1          // Analog input pin connect to temperature sensor OBJ pin
#define BTN_MEAS          12          // Button input pin to send information via BLE when pressed
#define LED_MEAS          13          // LED output pin to show that measurement is taken for BLE
#define BUZZ_MEAS         17          // Buzzer output pin to show measurement complete

// Macro definitions - comment out CALIB + DEBUG if not required
// #define CALIB                                               // for calibrating sensor
// #define DEBUG                                               // for debug messages
#define ArrayCount(array) (sizeof array / sizeof array[0])  // for determining 2-dim array sizes
 
// Health Thermometer date time data structure
// This is an optional feature and not included by default
/*
 struct BLEdateTime {
  uint16_t Year;
  uint8_t Month;
  uint8_t Day;
  uint8_t Hours;
  uint8_t Mins;
  uint8_t Secs;
};

 */

// Health thermometer location definitions
const uint8_t
  BLE_HTS_TEMP_TYPE_ARMPIT =            1,
  BLE_HTS_TEMP_TYPE_BODY =              2,
  BLE_HTS_TEMP_TYPE_EAR =               3,
  BLE_HTS_TEMP_TYPE_FINGER =            4,
  BLE_HTS_TEMP_TYPE_GI_TRACT =          5,
  BLE_HTS_TEMP_TYPE_MOUTH =             6,
  BLE_HTS_TEMP_TYPE_RECTUM =            7,
  BLE_HTS_TEMP_TYPE_TOE =               8,
  BLE_HTS_TEMP_TYPE_EAR_DRUM =          9;

// Add in the resistance values for negative temperatures
// Min temperature is -10℃ and Max temperature is 100℃
const long RT[110] = {
  531958,504588,478788,454457,431504,409843,389394,370082,351839,334598,
  318300,302903,288329,274533,261471,249100,237381,226276,215750,205768,
  196300,187316,178788,170691,163002,155700,148766,142183,135936,130012,
  124400,119038,113928,109059,104420,100000,95788,91775,87950,84305,
  80830,77517,74357,71342,68466,65720,63098,60595,58202,55916,
  53730,51645,49652,47746,45924,44180,42511,40912,39380,37910,
  36500,35155,33866,32631,31446,30311,29222,28177,27175,26213,
  25290,24403,23554,22738,21955,21202,20479,19783,19115,18472,
  17854,17260,16688,16138,15608,15098,14608,14135,13680,13242,
  12819,12412,12020,11642,11278,10926,10587,10260,9945,9641,
  9347,9063,8789,8525,8270,8023,7785,7555,7333,7118
};

const float OBJ [13][12]={
/*0*/             { 0,-0.274,-0.58,-0.922,-1.301,-1.721,-2.183,-2.691,-3.247,-3.854,-4.516,-5.236}, //
/*1*/             { 0.271,0,-0.303,-0.642,-1.018,-1.434,-1.894,-2.398,-2.951,-3.556,-4.215,-4.931},  //→surrounding temperature,from -10,0,10,...100
/*2*/             { 0.567,0.3,0,-0.335,-0.708,-1.121,-1.577,-2.078,-2.628,-3.229,-3.884,-4.597},   //↓object temperature,from -10,0,10,...110
/*3*/             { 0.891,0.628,0.331,0,-0.369,-0.778,-1.23,-1.728,-2.274,-2.871,-3.523,-4.232},
/*4*/             { 1.244,0.985,0.692,0.365,0,-0.405,-0.853,-1.347,-1.889,-2.482,-3.13,-3.835},
/*5*/             { 1.628,1.372,1.084,0.761,0.401,0,-0.444,-0.933,-1.47,-2.059,-2.702,-3.403},
/*6*/             { 2.043,1.792,1.509,1.191,0.835,0.439,0,-0.484,-1.017,-1.601,-2.24,-2.936},
/*7*/             { 2.491,2.246,1.968,1.655,1.304,0.913,0.479,0,-0.528,-1.107,-1.74,-2.431},
/*8*/             { 2.975,2.735,2.462,2.155,1.809,1.424,0.996,0.522,0,-0.573,-1.201,-1.887},
/*9*/             { 3.495,3.261,2.994,2.692,2.353,1.974,1.552,1.084,0.568,0,-0.622,-1.301},
/*10*/            { 4.053,3.825,3.565,3.27,2.937,2.564,2.148,1.687,1.177,0.616,0,-0.673},
/*11*/            { 4.651,4.43,4.177,3.888,3.562,3.196,2.787,2.332,1.829,1.275,0.666,0},
/*12*/            { 5.29,5.076,4.83,4.549,4.231,3.872,3.47,3.023,2.527,1.98,1.379,0.72}
};


unsigned long t_buzz = 0L;

const float HW_OFFSETVOLTAGE = 0.500;             // This is the hardware defined offset voltage - don't change
const float CALIB_OFFSETVOLTAGE = 0.0175;         // This is a calibrated offset factor.

const float ANAREF_VALS[5] = { 2.23, 0.0, 1.0, 1.65, 2.23 };     // AR_INTERNAL, not used here, AR_INTERNAL1V0, AR_INTERNAL1V65, AR_INTERNAL2V23

// This is the object temperature band given in the 2-d array (as above)
const float TEMPBANDRANGE = 10.0;

// the min and max bounds of the 2-d array (from above)
const int MINTHERMTEMP = -10;
const int MAXTHERMTEMP = 100;
const int MINOBJ_MV = -10;
const int MAXOBJ_MV = 110;

// runtime calibrations - option for future use.
float calibVolt =   0.0;
float TempCalib = -0.2;

// this is the index for analog reference voltage selected
uint8_t anaRef_ind = 0;

byte btnState = 0;         // variable for reading the pushbutton status

float ave_temp = 0.0;         // this is the average object temperature value that will be sent via BLE
unsigned int temp_cnt = 0;

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BLEGatt gatt(ble);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */
int32_t htsServiceId;
int32_t htsMeasureCharId;
int32_t htsBodyLocId;


/* To store the date and time information (optional info for BLE service)*/
//BLEdateTime BLEDT;


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{

  pinMode(BTN_MEAS, INPUT);
  pinMode(LED_MEAS, OUTPUT);
  pinMode(BUZZ_MEAS, OUTPUT);

  digitalWrite(LED_MEAS, LOW);
  digitalWrite(BUZZ_MEAS, LOW);
  
  analogReference(AR_INTERNAL1V0);    //set the refenrence voltage 1.0V,the distinguishability can up to 1mV.

  boolean success;

  // Make sure to use the same as this is used to obtain reference voltage used.
  // This gives the index for the ANAREF_VALS array
  anaRef_ind = AR_INTERNAL1V0 - 1;

  delay(1000);
  
  // Take a couple of analog readings to stabilise sensor
  for (byte i = 0; i < 10; i++) {
    analogRead(SUR_TEMP_PIN);
    analogRead(OBJ_TEMP_PIN);
    delay(20);
  }
  delay(1000);
  
  #ifdef CALIB
    Serial.println("");
    Serial.println(F("Grove IR Temp Sensor Calibration"));
    Serial.println("");
    delay(1000);
    Serial.print(F("Calibrating with int voltage reference..."));
    Serial.print(ANAREF_VALS[anaRef_ind], 2);
    Serial.println("V");
    Serial.flush();
    calibSensorVoltage();
    Serial.println(F("Done..."));
    Serial.flush();
  #endif
  
  Serial.println("");
  Serial.println(F("Adafruit Bluefruit Health Thermometer Example"));
  Serial.println(F("----------------------------------------------"));
  Serial.flush();

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'BlufruiTHERMO"));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=BlufruiTHERMO")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding Health Thermometer Service definition (UUID = 0x1809): "));
  Serial.flush();
  htsServiceId = gatt.addService(0x1809);
  if (htsServiceId == 0) {
    error(F("Could not add Thermometer service"));
  }
  
  /* Add the Temperature Measurement characteristic which is composed of
   * 1 byte flags + 4 bytes for float + any optional extras such as date-timestamp and temperature location*/
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding Temperature Measurement characteristic (UUID = 0x2A1C): "));
  Serial.flush();
  htsMeasureCharId = gatt.addCharacteristic(0x2A1C, GATT_CHARS_PROPERTIES_INDICATE, 5, 13, BLE_DATATYPE_BYTEARRAY);
  if (htsMeasureCharId == 0) {
    error(F("Could not add Temperature Measurement characteristic"));
  }

  Serial.println(F("Adding Optional Temperature Type characteristic (UUID = 0x2A1D): "));
  Serial.flush();
  htsBodyLocId = gatt.addCharacteristic(0x2A1D, GATT_CHARS_PROPERTIES_READ, 1, 1, BLE_DATATYPE_BYTEARRAY);
  if (htsBodyLocId == 0) {
    error(F("Could not add Temperature Type characteristic"));
  }
  
  // Set the body location - needs to be within an array
  uint8_t TempLocation[1] = {BLE_HTS_TEMP_TYPE_BODY};
  gatt.setChar(htsBodyLocId, TempLocation, sizeof(TempLocation));

  /* Add the Health Thermometer Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Health Thermometer Service UUID to the advertising payload: "));
  Serial.flush();
  uint8_t advdata[] { 0x02, 0x01, 0x06, 0x05, 0x02, 0x09, 0x18, 0x0a, 0x18 };
  ble.setAdvData( advdata, sizeof(advdata) );

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();
  delay(500);
  Serial.println();
  Serial.println(F("Ready to take measurements..."));
  Serial.println();
  Serial.flush(); 

  t_buzz = 0;
  
}

/** Send randomized heart rate data continuously **/
void loop(void)
{
  // read the state of the pushbutton value:
  btnState = !digitalRead(BTN_MEAS);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (btnState == HIGH) {
    if (!t_buzz) {
      // turn LED on:
      digitalWrite(LED_MEAS, HIGH);
      t_buzz = millis();
    }
    if (t_buzz) {
      if ((millis() - t_buzz) <= 2000) {
        // Take termperature measurements
        measureTemp();        // get the temperature measruements from the sensor
      }
      else {
        digitalWrite(BUZZ_MEAS, HIGH);
      }
    }
    
  } else {
    if (t_buzz) {
      // turn LED off:
      digitalWrite(LED_MEAS, LOW);
      digitalWrite(BUZZ_MEAS, LOW);
      t_buzz = 0;

      // calculate ave temp and update the Bluetooth LE GATT Service Characteristic
      if (temp_cnt) {
        ave_temp = ave_temp / temp_cnt;
        Serial.print(F("AVERAGE Tobj: "));
        Serial.println(ave_temp,2);
        Serial.flush();

        // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.temperature_measurement.xml
        // Chars value is 1 flag + 4 float value. Tempearature is in Fahrenheit unit (set bit(0))
        uint8_t temp_measurement[4] = { '\0' }; //Tempearature is in Centigrade unit
        uint8_t BLE_measurement[6] = { '\0' }; //Tempearature is in Centigrade unit
        BLE_measurement[0] = B00000100;

        float2IEEE11073(double(ave_temp), temp_measurement);
        
        for (byte xx = 0; xx < 4; xx++) BLE_measurement[xx+1] = temp_measurement[xx];
        BLE_measurement[5] = BLE_HTS_TEMP_TYPE_BODY;     // temperature location is body general
        
        gatt.setChar(htsMeasureCharId, BLE_measurement, sizeof(BLE_measurement));

        ave_temp = 0.0;
        temp_cnt = 0;
     }
    }
  }
}

// ==========================================================
// this function used to determine if calibration of thermistor required
// ==========================================================

void calibSensorVoltage() {
  float cumVoltages = 0.0;
  int objValue = 0;
  for (byte xx = 0; xx < 200; xx++) {
    // read the values from the sensor:
    objValue = analogRead(OBJ_TEMP_PIN); // discard first reading
    delay(20); 
    objValue = analogRead(OBJ_TEMP_PIN);
    for (byte i = 0; i< 4; i++) {
      delay(20);
      objValue += analogRead(OBJ_TEMP_PIN);
    }
    objValue /= 5;
  
    // convert to voltage and add
    cumVoltages += (objValue*ANAREF_VALS[anaRef_ind]/1023.0);
    delay(100);
  }

  // Now save the average less the offset
  calibVolt = (cumVoltages/200.0) - (HW_OFFSETVOLTAGE + CALIB_OFFSETVOLTAGE);
  if (calibVolt*100 > 0.5) {
    Serial.print(F("Add this to Calibrated Offset Voltage: "));
    Serial.println(calibVolt, 4);
  }
  else Serial.println(F("Sensor voltage calibration fine"));
}

// ==========================================================
// this function used to get the thermistor (surrounding temp) value
// ==========================================================

float binSearch(long Rval)
{
  float tmid = 0.0;
  int lo = 0;
  int hi = sizeof(RT)/sizeof(long);
  int mid = 0;
  #ifdef DEBUG
    Serial.print(F("Array size: ")); Serial.print(hi);
  #endif

  while (lo <= hi) {
    mid = (lo + hi)/2;
    if(Rval < RT[mid])
      lo = mid+1;
    else  //(x>=RT[mid])
      hi = mid-1;
  }
  tmid = float(mid+MINTHERMTEMP) + (float(RT[mid]-Rval)/float(RT[mid]-RT[mid+1]));
  Serial.print(F("Thermistor: "));
  Serial.print(tmid,1);
  return tmid;
}


// ==============================================================
// this function determines the object temperature from thermopile
// sensor based on the thermistor value and 2-d array lookup table
// ==============================================================

float arraysearch(float th, float mv)
{
  int lowmv = 0;
  int highmv = ArrayCount(OBJ);
  int thermInd = int(th/10.0) + 1;

  float Offset = (th - (thermInd * 10 + MINTHERMTEMP))/10.0;
  float tCoef[2] = {'\0'};
  float tmid = 0.0;
  
  byte i = 0;
  #ifdef DEBUG
    Serial.print(thermInd);  Serial.print(F(" -- offset ")); Serial.println(Offset);
  #endif

  for (i = 0; i < 13 ; i++) {
    if (mv < (OBJ[i][thermInd] - ((OBJ[i][thermInd] - OBJ[i][thermInd+1])*Offset))) {
      tCoef[1] = OBJ[i][thermInd] - ((OBJ[i][thermInd]-OBJ[i][thermInd+1])*Offset);
      break;
    }
    tCoef[0] = OBJ[i][thermInd] - ((OBJ[i][thermInd]-OBJ[i][thermInd+1])*Offset);
  }

  Offset = (mv - tCoef[0]) / (tCoef[1] - tCoef[0]);
  #ifdef DEBUG
    Serial.print(F("Index ")); Serial.print(i-1);  Serial.print(F(" & ")); Serial.print(i);
    Serial.print(F(" | mv Offset ")); Serial.print(Offset);
  #endif
  
  Serial.print(F(" | Thermopile: "));
  tmid = (i-1)*TEMPBANDRANGE + float(MINTHERMTEMP) + TEMPBANDRANGE * Offset + TempCalib;
  Serial.println(tmid,1);
  return tmid;
}

void measureTemp() {
    // This determines the thermistor temperature value
    // and then calculates the thermopile temperature value
    long Rval =         0L;
    float tmpVolt =     0.0;
    float thermistor =  0.0;
    float Obj_temp =    0.0;
    
    int anaValue = 0;
    
    // taking measurements from the thermistor
    // ---------------------------------------
    anaValue = analogRead(SUR_TEMP_PIN); // discard first reading
    delay(20); 
    anaValue = analogRead(SUR_TEMP_PIN);
    for (byte i = 0; i< 9; i++) {
      delay(20);
      anaValue += analogRead(SUR_TEMP_PIN);
    }
    anaValue /= 10;
    tmpVolt = (anaValue*ANAREF_VALS[anaRef_ind]/1023.0);

    #ifdef DEBUG
      Serial.print(F("surVal: ")); Serial.print(anaValue);
      Serial.print(F(" -- surVolt: ")); Serial.println(tmpVolt);
    #endif
    
    Rval = long(2000000*tmpVolt/(2.50-tmpVolt));     // Rseries in circuit = 2M ohms and 2.5V is Vref in circuit
    thermistor=binSearch(Rval);

    // taking measurements from the thermopile sensor
    // ----------------------------------------------
    anaValue = analogRead(OBJ_TEMP_PIN); // discard first reading
    anaValue = analogRead(OBJ_TEMP_PIN);
    for (byte i = 0; i< 9; i++) {
      delay(20);
      anaValue += analogRead(OBJ_TEMP_PIN);
    }
    anaValue /= 10;
    // Now determine millivolts (scale by 10? to ensure correct result)
    tmpVolt = ((anaValue*ANAREF_VALS[anaRef_ind]/1023.0)-(HW_OFFSETVOLTAGE + CALIB_OFFSETVOLTAGE))*10.0;

    #ifdef DEBUG
      Serial.print(F("objVal: ")); Serial.print(anaValue);
      Serial.print(F(" -- milliVolt: ")); Serial.println(tmpVolt);
    #endif
    
    Obj_temp=arraysearch(thermistor, tmpVolt);

    if((Obj_temp>100)||(Obj_temp<=-10)) {
      Serial.println (F("Object temperature out of range!"));
    }
    else {
      if (t_buzz) {
        ave_temp += Obj_temp;
        temp_cnt++;
      }
    }
}
