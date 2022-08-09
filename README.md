# lilygo-t-sim700g-asset-tracker-example
This is a simple example for using the Lilygo T-SIM7000G as an asset tracker.

To use this code you only need to set a few parameters in the code:

```// Your GPRS/Cellular APN and credentials, if any
const char apn[]  = "";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your endpoint server details
const char server[] = "yourhost.yourdomain.com";
const char resource[] = "/your-endpoint";
const int  port = 80;
```
There are several other parameters that can easily be tweaked and adjusted for your application.
