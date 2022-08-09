/**************************************************************************************
 * 
 *  Lilygo T-SIM7000G Tracker Example
 *  Version 2.2
 *  Last updated 2022-08-09
 *  Written by Scott C. Lemon
 *  Based on Lilygo Sample Code
 * 
 **************************************************************************************/

#define TINY_GSM_MODEM_SIM7000

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Test Flags from original Lilygo/TinyGSM code
#define TINY_GSM_TEST_GPRS true
#define TINY_GSM_TEST_WIFI false
#define TINY_GSM_TEST_CALL false
#define TINY_GSM_TEST_SMS false
#define TINY_GSM_TEST_USSD false
#define TINY_GSM_TEST_BATTERY false
#define TINY_GSM_TEST_GPS true

// powerdown modem after tests
#define TINY_GSM_POWERDOWN false

// do we want to post to out defined endpoint?
#define POST_TO_ENDPOINT  true

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS/Cellular APN and credentials, if any
const char apn[]  = "";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your endpoint server details
const char server[] = "yourhost.yourdomain.com";
const char resource[] = "/your-endpoint";
const int  port = 80;

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
HttpClient http(client, server, port);


#define uS_TO_S_FACTOR 1000000ULL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP     600       /* Time ESP32 will go to sleep (in seconds) */
#define POWER_DOWN_DELAY  180       /* Time ESP32 will delay before power down (in seconds) */
#define TRANSMIT_INTERVAL 120       /* How often should the tracker transmit the current location */

#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4

#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13
#define LED_PIN     12

// variables ...
float lat,  lon;
float lat2      = 0;
float lon2      = 0;
float speed2    = 0;
float alt2      = 0;
int   vsat2     = 0;
int   usat2     = 0;
float accuracy2 = 0;
int   year2     = 0;
int   month2    = 0;
int   day2      = 0;
int   hour2     = 0;
int   min2      = 0;
int   sec2      = 0;

uint32_t  secondsWithoutNetwork = 0;
uint32_t  lastTransmitTimestamp = 0;
boolean   networkConnected = false;
boolean   networkConnectedChanged = true;
uint8_t   dotCounter = 0;
#define   DOT_EVERY_SECONDS   30

/**************************************************************************************
 * 
 * Setup
 * 
 **************************************************************************************/
void setup()
{
    // Set console baud rate
    SerialMon.begin(115200);
    delay(10);

    // Set LED OFF
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Reset Modem
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(300);
    digitalWrite(PWR_PIN, LOW);

    // Check if we have a SDCard
    // Note: Even if one is inserted we are currently not using this
    //       It could be used to store time stamped data for when we get back into coverage
    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    Serial.println("Checking for SDCard ...");
    if (!SD.begin(SD_CS)) {
        Serial.println("SDCard MOUNT FAIL - No SDCard Installed?");
    } else {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        String str = "SDCard Size: " + String(cardSize) + "MB";
        Serial.println(str);
    }

    // Lilygo had this in their examples
    DBG("Wait 6 seconds for some reason ...");
    delay(6000);

    // Configure modem serial interface
    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    DBG("Initializing modem ...");
    if (!modem.restart()) {
        DBG("Failed to restart modem, delaying 10s and retrying");
        // return;
    }

    // Display modem name and information
    String name = modem.getModemName();
    delay(500);
    DBG("Modem Name:", name);
    String modemInfo = modem.getModemInfo();
    delay(500);
    DBG("Modem Info:", modemInfo);

    // Unlock your SIM card with a PIN if needed
    if ( GSM_PIN && modem.getSimStatus() != 3 ) {
        modem.simUnlock(GSM_PIN);
        DBG("SIM Check ...");
    }
    DBG("... SIM Good!");

/**************************************************************************************
 * 
 * Configure the modem network mode and preferred mode
 * 
 **************************************************************************************
 *
 *  Set Network mode:
 *     2 Automatic
 *    13 GSM only
 *    38 LTE only
 *    51 GSM and LTE only
 *    
 **************************************************************************************/
    bool boolRes;
    DBG("Setting network mode ...");
    do {
        boolRes = modem.setNetworkMode(51);
        delay(1000);
    } while (boolRes != true);
    DBG("... done!");

/**************************************************************************************
 * 
 *  Set Preferred mode:
 *    1 CAT-M
 *    2 NB-Iot
 *    3 CAT-M and NB-IoT
 *    
 **************************************************************************************/
    DBG("Setting Cat-M only ...");
    
    do {
        boolRes = modem.setPreferredMode(1);
        delay(1000);
    } while (boolRes != true);
    DBG("... done!");

/**************************************************************************************
 * 
 *  Reset the GPS
 *  
 **************************************************************************************/
    DBG("Power off GPS ...");
    // Set SIM7000G GPIO4 LOW ,turn off GPS power
    // CMD:AT+SGPIO=0,4,1,0
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,0");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,0 false ");
    }
    // Set SIM7000G GPIO4 HIGH ,Open GPS power
    // CMD:AT+SGPIO=0,4,1,1
    // Only in version 20200415 is there a function to control GPS power
    DBG("Power on GPS ...");
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,1 false ");
    }
    modem.enableGPS();
    DBG("... done!");

  // we want the device to transmit immediately upon boot so set the last TX in the past
  lastTransmitTimestamp = millis() - TRANSMIT_INTERVAL * 1000;

  // entering the loop!
  DBG("Entering main loop!");

}
/**************************************************************************************
 * 
 * Loop
 * 
 **************************************************************************************/
void loop()
{

  /**************************************************************************************
   * 
   *  On each loop we want to detect if we are connected or not.
   *  This will be used to log how long we were out of coverage
   *  
   **************************************************************************************/
    if (modem.isNetworkConnected()) {
        if (!networkConnected) {
          DBG("Network connected");
          networkConnected = true;
          networkConnectedChanged = true;
        } else {
          networkConnectedChanged = false;          
        }
    } else {
        if (networkConnected) {
          DBG("!isNetworkConnected");
          networkConnected = false;
          networkConnectedChanged = true;
        } else {
          networkConnectedChanged = false;          
        }
        delay(1000);
        secondsWithoutNetwork++;
        return;      
    }

    // if we lost the cellular connection try to force a reconnect
    if (!networkConnected && networkConnectedChanged) {
      modem.gprsConnect(apn, gprsUser, gprsPass);
    }

    // wait until we get the network back, and count for how long
    if (!modem.waitForNetwork()) {
        DBG("!waitForNetwork");
        delay(1000);
        secondsWithoutNetwork++;
        return;
    } else {
      // delay here just to slow loop and print dots ...
      dotCounter++;
      if (dotCounter == DOT_EVERY_SECONDS) {
        Serial.println("Transmit every: " + String(TRANSMIT_INTERVAL) + "  Waited " + String(DOT_EVERY_SECONDS) + " seconds ...");
        //DBG("Waited 30 seconds ...");
        dotCounter = 0;
      }
      delay(1000);
    }

    if (networkConnectedChanged) {
      IPAddress local = modem.localIP();
      DBG("Local IP:", local);
    }

    // is it time to transmit?
    if ((millis() - lastTransmitTimestamp) > TRANSMIT_INTERVAL * 1000) {

      // reset the last transmit timestamp
      lastTransmitTimestamp = millis();
      
      DBG("Connecting to", apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
          DBG("!gprsConnect");
          delay(1000);
          secondsWithoutNetwork++;
          return;      
      }
  
      bool res = modem.isGprsConnected();
      DBG("GPRS status:", res ? "connected" : "not connected");
      
// TODO: what do we do here if it failed above?
  
      // display the SIM and connection information
      String ccid = modem.getSimCCID();
      DBG("CCID:", ccid);
  
      String imsi = modem.getIMSI();
      DBG("IMSI:", imsi);
    
      String imei = modem.getIMEI();
      DBG("IMEI:", imei);
  
      String cop = modem.getOperator();
      DBG("Operator:", cop);
  
      IPAddress local = modem.localIP();
      DBG("Local IP:", local);
  
      int csq = modem.getSignalQuality();
      DBG("Signal quality:", csq);
  
      // check GPS and get our GPS data
      while (1) {
        int noLockCount = 0;
        if (modem.getGPS(&lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
                      &year2, &month2, &day2, &hour2, &min2, &sec2)) {
          DBG("Latitude:", String(lat2, 8), "\tLongitude:", String(lon2, 8));
          DBG("Speed:", speed2, "\tAltitude:", alt2);
          DBG("Visible Satellites:", vsat2, "\tUsed Satellites:", usat2);
          DBG("Accuracy:", accuracy2);
          DBG("Year:", year2, "\tMonth:", month2, "\tDay:", day2);
          DBG("Hour:", hour2, "\tMinute:", min2, "\tSecond:", sec2);
          break;
        } else {
          DBG("Waiting for GPS lock ...");
          noLockCount++;
          // if it's been 2 minutes ... reset the GPS!
          if (noLockCount > 60) {
                DBG("Reset GPS ... powering down ...");
                // Set SIM7000G GPIO4 LOW ,turn off GPS power
                // CMD:AT+SGPIO=0,4,1,0
                // Only in version 20200415 is there a function to control GPS power
                modem.sendAT("+SGPIO=0,4,1,0");
                if (modem.waitResponse(10000L) != 1) {
                    DBG(" SGPIO=0,4,1,0 false ");
                }
                delay(1000);
                // Set SIM7000G GPIO4 HIGH ,Open GPS power
                // CMD:AT+SGPIO=0,4,1,1
                // Only in version 20200415 is there a function to control GPS power
                DBG("... power on GPS ...");
                modem.sendAT("+SGPIO=0,4,1,1");
                if (modem.waitResponse(10000L) != 1) {
                    DBG(" SGPIO=0,4,1,1 false ");
                }
                modem.enableGPS();
                DBG("... done!");
          }
          //Serial.print("getGPS ");
          //Serial.println(millis());
        }
        delay(2000);
      }

      // transform and adjust data values
      //
      // Note: speed is sometimes negative for some reason, set it to zero
      if (speed2 < 0) {
        speed2 = 0;
      }
      //  metric -> imperial
      //  kph -> mph
      speed2 = speed2 * 0.621371;
      //  meters -> feet
      alt2 = alt2 * 3.28084;
      
      // post to endpoint ...
      if (POST_TO_ENDPOINT) {
        DBG("POSTing data to endpoint ... ");
        String contentType = "application/json";
  
        String postData = "{"
          "\"version\":1.1,"
          "\"sensorType\":\"Wovyn LG7000G\","
          "\"emitterGUID\":\"" + imei + "\","
          "\"iccid\":\"" + ccid + "\","
          "\"secondsWithoutNetwork\":" + String(secondsWithoutNetwork) + ","
          "\"imsi\":\"" + imsi + "\","
          "\"operator\":\"" + cop + "\","
          "\"signal\":" + String(csq,8) + ","
          "\"lat\":" + String(lat2,8) + ","
          "\"lon\":" + String(lon2,8) + ","
          "\"speed\":" + String(speed2,3) + ","
          "\"alt\":" + String(alt2,3) + ","
          "\"vSat\":" + String(vsat2) + ","
          "\"uSat\":" + String(usat2) + ","
          "\"accuracy\":" + String(accuracy2,3) + ""
          "}";
      
        http.post(resource, contentType, postData);
      
        // read the status code and body of the response
        int statusCode = http.responseStatusCode();
        String response = http.responseBody();
      
        Serial.print("Status code: ");
        Serial.println(statusCode);
        Serial.print("Response: ");
        Serial.println(response);
        DBG("... end POST!");
    
        // stop the http client
        http.stop();

        // reset variables after successful POST
        secondsWithoutNetwork = 0;

      } // end post to endpoint
    } // end if it's time to transmit

// at this time we do NOT want to disconnect the cellular connection
#if FALSE
    modem.gprsDisconnect();
    if (!modem.isGprsConnected()) {
        DBG("GPRS disconnected");
    } else {
        DBG("GPRS disconnect: Failed.");
    }
#endif

// we also do NOT want to power down the modem
#if TINY_GSM_POWERDOWN
    // Try to power-off (modem may decide to restart automatically)
    // To turn off modem completely, please use Reset/Enable pins
    modem.poweroff();
    DBG("Poweroff modem.");
#endif

// ... and loop!
}
