# lilygo-t-sim7000g-asset-tracker-example
This is a simple example for using the Lilygo T-SIM7000G as an asset tracker.

To use this code you need to set several parameters in the code:

```/*
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

```
There are several other parameters that can easily be tweaked and adjusted for your application.
