/**************************************************************************************
 * 
 *  Lilygo T-SIM7000G Tracker Example
 *  Version 3.0
 *  Last updated 2022-10-01
 *  Written by Scott C. Lemon
 *  Based on Lilygo Sample Code
 * 
 * v3.0 - added SSL and MQTT examples
 *      - From: https://github.com/vshymanskyy/TinyGSM#which-version-of-the-sim7000-code-to-use
 *          "There are two versions of the SIM7000 code, one using TINY_GSM_MODEM_SIM7000 and another 
 *          with TINY_GSM_MODEM_SIM7000SSL. The TINY_GSM_MODEM_SIM7000 version does not support SSL 
 *          but supports up to 8 simultaneous connections. The TINY_GSM_MODEM_SIM7000SSL version 
 *          supports both SSL and unsecured connections with up to 2 simultaneous connections. 
 *          So why are there two versions? The "SSL" version uses the SIM7000's "application" commands 
 *          while the other uses the "TCP-IP toolkit". Depending on your region/firmware, one or the 
 *          other may not work for you. Try both and use whichever is more stable. If you do not need 
 *          SSL, I recommend starting with TINY_GSM_MODEM_SIM7000."
 *      - major code reorganization
 *      - added getting network time
 *      - refactored the "time to send" waiting code and messages
 *      - fix to secondsWithoutNetwork on first run
 * TODO
 *      - Modem Power down and Module deep sleep still not complete 
 * 
 **************************************************************************************/

/*
 * Set serial for debug console (to the Serial Monitor, default speed 115200)
 */
#define SerialMon Serial

/*
 * Set serial for AT commands (to the module)
 * Use Hardware Serial on Mega, Leonardo, Micro
 */
#define SerialAT Serial1

/*
 * See all AT commands, if wanted
 */
// #define DUMP_AT_COMMANDS

/*
 * Define the serial console for debug prints, if needed
 */
#define TINY_GSM_DEBUG SerialMon

/*
 * powerdown modem after tests?
 */
#define TINY_GSM_POWERDOWN false

/*
 * do we want to post to our defined endpoints?
 */
#define POST_TO_ENDPOINT  true

/*
 * set GSM PIN, if any
 */
#define GSM_PIN ""

/*
 * We want to use the ESP32 RTC Memory to keep count of deep sleep cycles
 */
RTC_DATA_ATTR int deepSleeps = 0;

/*
 * Your cellular definitions
 *   - You must provide your GPRS/Cellular APN and credentials, if any
 */
const char apn[]  = "super";
const char gprsUser[] = "";
const char gprsPass[] = "";

/*
 * Your endpoint configuration(s)
 *   - You must set the defined variables for your endpoints
 *   - Enable HTTP and/or MQTT
 *   - Indicate if you want SSL/TLS used for either
 */
// Your HTTP(S) server details
#define ENABLE_HTTP false
#define USE_HTTPS   false
const char server[] = "";
const char resource[] = "";
const int  port = 80;

// Your MQTT(S) broker details
#define ENABLE_MQTT true
#define USE_MQTTS   false
const char broker[] = "test.mosquitto.org";
const int  mqttPort = 1883;

// this will select the correct driver to use
#if (ENABLE_HTTP && USE_HTTPS) || (ENABLE_MQTT && USE_MQTTS)
  #define TINY_GSM_MODEM_SIM7000SSL   // provides both SSL and non-SSL connections
#else
  #define TINY_GSM_MODEM_SIM7000
#endif

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#include <ArduinoMqttClient.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <SPI.h>
#include <SD.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#if ENABLE_HTTP
  // NOTE: These HTTP errors are 0 and negatives, and so abs() must be used to print correctly
  String httpConnectError[] = {
    "HTTP_SUCCESS",
    "HTTP_ERROR_CONNECTION_FAILED",
    "HTTP_ERROR_API",
    "HTTP_ERROR_TIMED_OUT",
    "HTTP_ERROR_INVALID_RESPONSE"
    };

  // HTTP is using modem connection 0
  #if USE_HTTPS
    TinyGsmClientSecure http_client(modem,0);
    HttpClient http(http_client, server, port);
  #else
    TinyGsmClient http_client(modem,0);
    HttpClient http(http_client, server, port);
  #endif
#endif

#if ENABLE_MQTT
  // NOTE: These MQTT connect errors start at -2 and so must be offset by 2 to print correctly
  String mqttConnectError[] = {
    "MQTT_CONNECTION_REFUSED",
    "MQTT_CONNECTION_TIMEOUT",
    "MQTT_SUCCESS",
    "MQTT_UNACCEPTABLE_PROTOCOL_VERSION",
    "MQTT_IDENTIFIER_REJECTED",
    "MQTT_SERVER_UNAVAILABLE",
    "MQTT_BAD_USER_NAME_OR_PASSWORD",
    "MQTT_NOT_AUTHORIZED"
    };

  // MQTT is using modem connection 1
  #if USE_MQTTS
    TinyGsmClientSecure mqtt_client(modem,1);
    MqttClient mqtt(mqtt_client);
  #else
    TinyGsmClient mqtt_client(modem,1);
    MqttClient mqtt(mqtt_client);
  #endif
#endif

#define uS_TO_S_FACTOR 1000000ULL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP       600     /* Time ESP32 will go to sleep (in seconds) */
#define POWER_DOWN_DELAY    180     /* Time ESP32 will delay before power down (in seconds) */

#define TRANSMIT_INTERVAL   120     /* How often should the tracker transmit the current location */

#define GPS_NO_LOCK_DELAY   2       /* How long to wait, looping to regain GPS lock, in seconds */
#define GPS_NO_LOCK_TIMEOUT 30      /* How long to wait, total, before resetting the GPS power */

// pins for the modem connection
#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4

// pins for the SD Card slot
#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13
#define LED_PIN     12

// Modem Network Modes
#define NM_AUTOMATIC     2
#define NM_GSM_ONLY     13
#define NM_LTE_ONLY     38
#define NM_GSM_AND_LTE  51

// Modem Preferred Modes (Category of Service)
#define PM_CAT_M            1
#define PM_NB_IOT           2
#define PM_CAT_M_AND_NB_IOT 3

// Modem RAT Types
String ratType[] = {"GSM","GSM Compact","UTRAN","GSM w/EGPRS","UTRAN w/HSDPA","UTRAN w/HSUPA","UTRAN w/HSDPA and HSUPA","E-UTRAN"};

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

uint32_t  gotNetwork = millis();
uint32_t  lostNetwork = millis();
uint32_t  secondsWithoutNetwork = -1;
uint32_t  lastTransmitTimestamp = 0;
boolean   networkConnected = false;
boolean   networkConnectedChanged = false;
uint32_t  waitingStarted = 0;
uint32_t  lastWaitingMessage = 0;
#define   WAITING_MESSAGE_EVERY_SECONDS   15

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

    Serial.println("Beginning Asset Tracker setup() ...");

    // Set LED OFF
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Reset Modem
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(300);
    digitalWrite(PWR_PIN, LOW);

    /*
     * Check if we have a SDCard
     * Note: Even if one is inserted we are currently not using this
     *       It could be used to store time stamped data for when we get back into coverage
     */
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
    DBG("Wait 10 seconds for modem to boot ...");
    delay(10000);

    // Configure modem serial interface
    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    DBG("Initializing modem ...");
    if (!modem.init()) {
        DBG("Failed to init modem? Continuing!");
        // return;
    }

    // Display modem name and information
    String name = modem.getModemName();
    delay(1000);
    DBG("Modem Name:", name);
    
    String modemInfo = modem.getModemInfo();
    delay(1000);
    DBG("Modem Info:", modemInfo);

    String firmware;
    modem.sendAT("+CGMR");
    if (modem.waitResponse(10000L, firmware) != 1) {
      DBG("Firmware: FAILED");
    } else {
      firmware.replace("\r\n" "OK" "\r\n", "");
      firmware.replace("_", " ");
      firmware.trim();
      DBG("Firmware: ", firmware);
    }

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
 *    NM_AUTOMATIC     2
 *    NM_GSM_ONLY     13
 *    NM_LTE_ONLY     38
 *    NM_GSM_AND_LTE  51
 *    
 **************************************************************************************/
    bool boolRes;
    DBG("Setting network mode ...");
    do {
        boolRes = modem.setNetworkMode(NM_GSM_AND_LTE);
        delay(1000);
    } while (boolRes != true);
    DBG("... done!");

/**************************************************************************************
 * 
 *  Set Preferred mode:
 *    PM_CAT_M            1
 *    PM_NB_IOT           2
 *    PM_CAT_M_AND_NB_IOT 3
 *    
 **************************************************************************************/
    DBG("Setting Cat-M only ...");
    
    do {
        boolRes = modem.setPreferredMode(PM_CAT_M);
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
          DBG("Network changed to connected!");
          gotNetwork = millis();
          if (secondsWithoutNetwork == -1) {
            // this is to deal with the first run of the loop
            secondsWithoutNetwork = 0;
          } else {
            // this is for any time after the first run of the loop
            secondsWithoutNetwork = lostNetwork - gotNetwork;            
          }
          networkConnected = true;
          networkConnectedChanged = true;
        } else {
          //DBG("Network still connected!");
          gotNetwork = millis();
          lostNetwork = millis();
          //secondsWithoutNetwork = lostNetwork - gotNetwork;
          networkConnectedChanged = false;          
        }
    } else {
        if (networkConnected) {
          DBG("Network changed to disconnected!");
          lostNetwork = millis();
          //secondsWithoutNetwork = lostNetwork - gotNetwork;
          networkConnected = false;
          networkConnectedChanged = true;
        } else {
          //DBG("Network still disconnected!");
          networkConnectedChanged = false;          
          lostNetwork = millis();
          secondsWithoutNetwork = lostNetwork - gotNetwork;
        }
        //delay(1000);
        //return;      
    }

    /*
     * if we lost the cellular connection try to force a reconnect
     * 
     * NOTE:  SIMCom indicated that a search can be forced by using AT+COPS=0
     *        This should trigger an immediate saerch again
     */
    if (!networkConnected && networkConnectedChanged) {
      DBG("Attempting to reconnect!");
      modem.sendAT("+COPS=0");
      //modem.restart();
      //modem.gprsConnect(apn, gprsUser, gprsPass);
    }

    /*
     * Check to see if we have network:
     *   Delay and return if we don't
     *   Print waiting to send messages if we do
     */
    if (!modem.waitForNetwork()) {
        DBG("!waitForNetwork");
        delay(1000);
        return;
    } else {
      // delay here just to slow loop and print dots ...
      if (waitingStarted == 0){
        waitingStarted = millis();
        lastWaitingMessage = waitingStarted;
      }

      if (((millis() - lastWaitingMessage) / 1000) >= WAITING_MESSAGE_EVERY_SECONDS) {
        Serial.println("Waited " + String((millis() - waitingStarted) / 1000) + " seconds of " + String(TRANSMIT_INTERVAL) + " second transmit interval ...");
        lastWaitingMessage = millis();

#if ENABLE_MQTT
        if(mqtt.connected()) {
          Serial.println("Poll MQTT Broker to keep connection alive!");
          mqtt.poll();
        }
#endif

      }
      delay(1000);
    }

    if (networkConnectedChanged) {
      IPAddress local = modem.localIP();
      DBG("Local IP:", local);
    }

    /*
     * is it time to transmit?
     */
    if ((millis() - lastTransmitTimestamp) >= TRANSMIT_INTERVAL * 1000) {

      DBG(">> Time to transmit!");
      DBG("Connecting to APN: ", apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
          DBG("!gprsConnect");
          delay(1000);
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

      String networkTime = modem.getGSMDateTime(DATE_FULL);
      DBG("Current Network Time:", networkTime);
      
      /************************************************************************
       * 
       * Possible values for access technology:
       * 
       *  0 GSM
       *  1 GSM Compact
       *  2 UTRAN
       *  3 GSM w/EGPRS
       *  4 UTRAN w/HSDPA
       *  5 UTRAN w/HSUPA
       *  6 UTRAN w/HSDPA and HSUPA
       *  7 E-UTRAN
       *  
       ************************************************************************/
      String copsString;
      String currentRatType;
      modem.sendAT("+COPS?");
      if (modem.waitResponse(10000L, copsString) != 1) {
        DBG("copsString: FAILED");
      } else {
        copsString.replace("\r\n" "OK" "\r\n", "");
        copsString.replace("_", " ");
        copsString.trim();
        DBG("copsString: ", copsString);
        currentRatType = ratType[String(copsString.charAt(copsString.length() - 1)).toInt()];
        DBG("RAT Type: ", currentRatType);
      }

      int csq = modem.getSignalQuality();
      DBG("Signal quality:", csq);
  
      /*
       * check GPS and get our GPS data
       */
      int noLockCount = 0;
      while (1) {
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
          noLockCount += GPS_NO_LOCK_DELAY;
          DBG("Delay in seconds = ", noLockCount);
          // if it's been GPS_NO_LOCK_TIMEOUT ... reset the GPS!
          if (noLockCount >= GPS_NO_LOCK_TIMEOUT) {
                noLockCount = 0;
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
          } // end if (noLockCount > GPS_NO_LOCK_TIMEOUT)
        } // else wait or reset GPS
        delay(GPS_NO_LOCK_DELAY * 1000);
      } // while forever waiting for GPS lock

      /*
       * transform and adjust data values
       * 
       * NOTE: speed is sometimes negative for some reason, set it to zero
       */
      if (speed2 < 0) {
        speed2 = 0;
      }
      //  metric -> imperial
      //  kph -> mph
      speed2 = speed2 * 0.621371;
      //  meters -> feet
      alt2 = alt2 * 3.28084;

#if ENABLE_HTTP
      // post to HTTP endpoint ...
      if (POST_TO_ENDPOINT && ENABLE_HTTP) {
        DBG("POSTing data to endpoint ... ");
        String contentType = "application/json";
  
        String postData = "{"
          "\"version\":1.1,"
          "\"sensorType\":\"Wovyn LG7000G\","
          "\"emitterGUID\":\"" + imei + "\","
          "\"iccid\":\"" + ccid + "\","
          "\"secondsWithoutNetwork\":" + String(secondsWithoutNetwork / 1000) + ","
          "\"imsi\":\"" + imsi + "\","
          "\"operator\":\"" + cop + "\","
          "\"signal\":" + String(csq,8) + ","
          "\"ratType\":\"" + currentRatType + "\","
          "\"networkTime\":\"" + networkTime + "\","
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
        if (statusCode < 0) {
          Serial.println(httpConnectError[abs(statusCode)]);
        }
        Serial.print("Response: ");
        Serial.println(response);
        DBG("... end POST!");
    
        // stop the http client
        http.stop();

        // reset variables after successful POST
        secondsWithoutNetwork = 0;

      } // end http post to endpoint
#endif

#if ENABLE_MQTT
      // update mqtt ...
      if (POST_TO_ENDPOINT && ENABLE_MQTT) {
        DBG("Updating MQTT ... ");

        if (!mqtt.connected()) {
          Serial.print("Attempting to connect to the MQTT broker: ");
          Serial.println(broker);
          
          if (!mqtt.connect(broker, mqttPort)) {
            Serial.print("MQTT connection failed! Error code = ");
            Serial.println(mqtt.connectError());
            Serial.println(mqttConnectError[mqtt.connectError()+2]);
            //mqtt.stop();
          
            return;
          }
        }
        
        Serial.println("Connected to the MQTT broker!");

        mqtt.poll();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/iccid");
        Serial.print("  Value: ");
        Serial.println(ccid);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/iccid");
        mqtt.print(ccid);
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/imsi");
        Serial.print("  Value: ");
        Serial.println(imsi);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/imsi");
        mqtt.print(imsi);
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/operator");
        Serial.print("  Value: ");
        Serial.println(cop);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/operator");
        mqtt.print(cop);
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/ratType");
        Serial.print("  Value: ");
        Serial.println(currentRatType);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/ratType");
        mqtt.print(currentRatType);
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/signal");
        Serial.print("  Value: ");
        Serial.println(String(csq,8));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/signal");
        mqtt.print(String(csq,8));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/networkTime");
        Serial.print("  Value: ");
        Serial.println(networkTime);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/networkTime");
        mqtt.print(networkTime);
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/secondWithoutNetwork");
        Serial.print("  Value: ");
        Serial.println(String(secondsWithoutNetwork / 1000));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/secondsWithoutNetwork");
        mqtt.print(String(secondsWithoutNetwork / 1000));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/latitude");
        Serial.print("  Value: ");
        Serial.println(String(lat2,8));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/latitude");
        mqtt.print(String(lat2,8));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/longitude");
        Serial.print("  Value: ");
        Serial.println(String(lon2,8));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/longitude");
        mqtt.print(String(lon2,8));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/speed");
        Serial.print("  Value: ");
        Serial.println(String(speed2,3));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/speed");
        mqtt.print(String(speed2,3));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/altitude");
        Serial.print("  Value: ");
        Serial.println(String(alt2,3));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/altitude");
        mqtt.print(String(alt2,3));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/vSat");
        Serial.print("  Value: ");
        Serial.println(String(vsat2));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/vSat");
        mqtt.print(String(vsat2));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/uSat");
        Serial.print("  Value: ");
        Serial.println(String(usat2));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/uSat");
        mqtt.print(String(usat2));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/accuracy");
        Serial.print("  Value: ");
        Serial.println(String(accuracy2,3));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/accuracy");
        mqtt.print(String(accuracy2,3));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/deepSleeps");
        Serial.print("  Value: ");
        Serial.println(deepSleeps);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/T-SIM7000G/AssetTracker/" + imei + "/deepSleeps");
        mqtt.print(deepSleeps);
        mqtt.endMessage();
        mqtt.flush();

        //mqtt.stop();
        //Serial.println("Stopped MQTT Connection");

      } // end mqtt updates
#endif

      Serial.println("... end of sending to endpoints!");

      // reset the last transmit timestamp and messages variables
      lastTransmitTimestamp = millis();
      waitingStarted = 0;
      lastWaitingMessage = 0;

      Serial.println("<< Waiting " + String(TRANSMIT_INTERVAL) + " seconds until next send!");

    } // end if it's time to transmit

// at this time we do NOT want to disconnect the cellular connection
#if FALSE
    DBG("Disconnecting GPRS ...");
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
