#pragma once

// Network information
#define WIFI_SSID "still_waters"
#define WIFI_PASSWORD "33turkeys511"

// #define DEVICE1
#define DEVICE2
// #define DEVICE3
// #define DEVICE4
// #define DEVICEDUCK

#ifdef DEVICE1
const char *writeAPIKey = "H6EVSHKVVKR8KNJP"; // battery channel
const char *readAPIKey = "VV1UZ2CRNCUJ8VER";  // control channel
long readChannelId = 597924;
long myChannelNumber = 592680;
#endif

#ifdef DEVICE2
const char *writeAPIKey = "VML7XDJTAELWPWH8"; // second battery channel
long myChannelNumber = 844390;                // channel 2
const char *readAPIKey = "OJXNM9UUX2V6P8UD";  // second control channel
long readChannelId = 844392;                  // channel2
#endif

#ifdef DEVICE3
const char *writeAPIKey = "0L6OFFAKEQTBBNBJ"; // third battery channel
long myChannelNumber = 1005405;               // channel 3
const char *readAPIKey = "BMKL2AZ5SB4SUNFZ";  // third control channel
long readChannelId = 1005406;                 // channel2
#endif

#ifdef DEVICE4
const char *writeAPIKey = "DXKLMJ1W7WMYDD8D"; // lone battery channel
long myChannelNumber = 10505221;              // channel 4
const char *readAPIKey = "P5SQIBO7FZNH9XBV";  // lone control channel
long readChannelId = 1050522;                 // channel2
#endif

#ifdef DEVICEDUCK
const char *writeAPIKey = "MEC37ONNJUT84A4U"; // Ducktank channel
long myChannelNumber = 1160823;               // channel 4
const char *readAPIKey = "CN409WRNXUVJYKGQ";  // Ducktank control channel
long readChannelId = 1160824;                 // channel2
#endif
