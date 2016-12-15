/* -------------------------------------
  IoT enabled Automatic Plant Irrigation System - IOT APIS2
  Based on ESP8622 NODEMCU v2 dev kit chip
   Code Version 0.1.0
   Parameters Version 01
  MM
  Change Log:
  2016-11-29
    v0.1.0 - work started

  ----------------------------------------*/

// TEST/DEBUG defines
// ------------------
#define _DEBUG_
#define _TEST_

// TaskScheduler options:
//#define _TASK_TIMECRITICAL    // Enable monitoring scheduling overruns
#define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass 
#define _TASK_STATUS_REQUEST  // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
//#define _TASK_WDT_IDS         // Compile with support for wdt control points and task ids
//#define _TASK_LTS_POINTER     // Compile with support for local task storage pointer
#define _TASK_PRIORITY          // Support for layered scheduling priority
//#define _TASK_MICRO_RES       // Support for microsecond resolutionMM
#include <TaskScheduler.h>

#include <EEPROM.h>
#include <AvgFilter.h>

#include <TimeLib.h>  // just including Time.h does not work anymore for now() method (for some reason)
#include <Timezone.h>
#include <RTClib.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
//#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <FS.h>

// Natural constants
#define US_TO_MM_FACTOR  (100000/583)
#define STATIC_TIME      1451606400UL

// Watering parameter defaults
// ---------------------------
#define RETRIES 2
#define RETRIES_MIN 1
#define RETRIES_MAX 10

// Time to run pump within one water run
#define WATERTIME 20 //Seconds
#define WATERTIME_MIN 5 //Seconds
#define WATERTIME_MAX 120 //Seconds

// Time to saturate
#define SATURATE 1 // 1 minute
#define SATURATE_MIN 1 // 1 minute
#define SATURATE_MAX 90 // 10 minutes

// % soil humidity to start pumping (low threshold)
#define NEEDWATER 65 // % to start pumping
#define NEEDWATER_MIN 20 // % to start pumping
#define NEEDWATER_MAX 75 // % to start pumping

// % soil humidity to stop pumping (high threshold)
#define STOPWATER 70 // % to stop pumping
#define STOPWATER_MIN 25 // % to stop pumping
#define STOPWATER_MAX 90 // % to stop pumping

// Hour of the day to "go to sleep" (i.e., do not operate after this hour)
#define GOTOSLEEP 22 // hour to go to sleep
#define GOTOSLEEP_MIN  0 // hour to go to sleep
#define GOTOSLEEP_MAX 24 // hour to go to sleep

// Hour of the day to "wake up" (i.e., operate after this hour)
#define WAKEUP    7 // hour to wake up
#define WAKEUP_MIN  0 // hour to wake up
#define WAKEUP_MAX 24 // hour to wake up

// Number of hours to add to wake up time on a weekend
#define WEEKENDADJ  3  // number of hours to add for the wakeup on a weekend
#define WEEKENDADJ_MIN  0  // number of hours to add for the wakeup on a weekend
#define WEEKENDADJ_MAX  12  // number of hours to add for the wakeup on a weekend

#define WLDEPTH   240 // mm, total depth of the water bucket
#define WLLOW     30  // mm, low level of water bucket

// Wifi
#define CONNECT_TIMEOUT 20  //seconds
#define NTP_TIMEOUT     20  //seconds
#define CONNECT_BLINK   260 // ms
#define WL_PERIOD       200 // ms

#define CONNECT_INTRVL  1000
#define NTPUPDT_INTRVL  1000

// PINs
// ----
// Moisture probe analog pin
#define MOISTURE_PIN      A0  // A0
#define MOISTURE_PWR_PIN  13  // D7
#define MOISTURE_GND_PIN  15  // D8
#define SENSOR_OUT_VALUE  1000

#define LEAK_PIN          02 // D4

// RGB LED
#define RGBLED_RED_PIN    10  // SD3 nodemcu pin
#define RGBLED_GREEN_PIN  05  // D1 nodemcu pin
#define RGBLED_BLUE_PIN   04  // D2 nodemcu pin

#define LEDON             PWMRANGE // full range 

// HC-SR04 ultrasound sensor
#define SR04_TRIGGER_PIN  14  // D5 nomemcu pin
#define SR04_ECHO_PIN     12  // D6 nomemcu pin

// PUMP pins
#define PUMP_PWR_PIN      00  // D3


// EEPROM token for parameters
const char *CToken = "APIS00\0"; // Eeprom token: Automatic Plant Irrigation System
const char *CSsid  = "Verizon-MiFi6620L-3D38-AMA\0";
const char *CPwd   = "21613718\0";
//const char *CSsid  = "beta_n\0";
//const char *CPwd   = "460w20thstret1973b\0";
const char *CDSsid  = "Plant_";
const char *CDPwd   = "changeme!";
const char *CHost  = "plant.io";
String ssid, pwd;

//long currentWaterLevel;
#ifndef _TEST_
#define  TICKER     TASK_HOUR
#define  SLEEP_TOUT (TASK_MINUTE * 2)

int   currentHumidity = 0;
int   currentWaterLevel = 0;
#else
#define  TICKER     (TASK_MINUTE * 3)
#define  SLEEP_TOUT TASK_MINUTE

int   currentHumidity = 60;
int   currentWaterLevel = 200;
#endif

#define TMEASURE_WATERTIME   5  // measure every 5 seconds while watering
#define MEASURE_PRIME        5  // 5 seconds
#define MEASURE_TIMES        3  // measure this number of times to average the value

const char* CWakeReasonSleep = "Deep-Sleep Wake";
const char* CWakeReasonReset = "External System";

// Forward definition of all callback methods is now required by v1.6.6
// ---------------------------------------------------------------------
void cfgInit();
void cfgLed();
void ledOnDisable();
void ntpUpdate();
void connectedChk();
void waterCallback();
bool waterOnenable();
void waterOndisable();
void measureWL();
bool measureWLOnEnable();
void measureMS();
void measureMSOnDisable();
void ticker();

void pingTOut();
void waterLevelCalc();
void measureRestart();

void waterCallback();
bool waterOnEnable();
void waterOnDisable();

void handleClientCallback();
void serverHandleRoot();
void serverHandleNotFound();

void sleepCallback();
void sleepTimeout();

void resetDevice();

// Task Scheduling
// ---------------
StatusRequest pulseComplete, measurementsReady, probeIdle, pageLoaded, sleepTime;
Scheduler ts, hpts;

#ifdef _TEST_
void testTicker();
Task tTest          (TASK_SECOND * 10, TASK_FOREVER, &testTicker, &ts, true);
#endif


Task tConfigure     (TASK_IMMEDIATE, TASK_ONCE, &cfgInit, &ts, true);
Task tLedBlink      (TASK_IMMEDIATE, TASK_FOREVER, &cfgLed, &ts, false, NULL, &ledOnDisable);
Task tTimeout       (TASK_IMMEDIATE, TASK_FOREVER, NULL, &ts);
Task tNtpUpdater    (NTPUPDT_INTRVL, NTP_TIMEOUT, &ntpUpdate, &ts, false);
Task tConnected     (TASK_SECOND, TASK_FOREVER, &connectedChk, &ts, false);

Task tHandleClients (TASK_SECOND, TASK_FOREVER, &handleClientCallback, &ts, false);

Task tTicker        (TICKER, TASK_FOREVER, &ticker, &ts, false);

Task tMesrLevel     (WL_PERIOD, TASK_FOREVER, &measureWL, &ts, false, &measureWLOnEnable, NULL);
Task tMesrMoisture  (TASK_SECOND, TASK_FOREVER, &measureMS, &ts, false, NULL, &measureMSOnDisable);

Task tMeasure       (&measureCallback, &ts); //, NULL, &measureOnDisable);
Task tMeasureRestart(&measureRestart, &ts);

Task tWater         (TASK_IMMEDIATE, TASK_ONCE, &waterCallback, &ts, false, &waterOnEnable, &waterOnDisable);

Task tPing          (TASK_IMMEDIATE, TASK_ONCE, &pingTOut, &hpts, false);
Task tWaterLevel    (&waterLevelCalc, &hpts);

Task tSleep         (&sleepCallback, &ts);
Task tSleepTimeout  (SLEEP_TOUT, TASK_ONCE, &sleepTimeout, &ts, false);

Task tReset         (&resetDevice, &ts);

// Parameters
// ----------
typedef struct {
  // token
  char      token[7];   //  6 digit token = APISxx, where xx is a version + '\0'

  // watering parameters
  byte      high;       //  high humidity mark - stop watering
  byte      low;        //  low humidity mark - start watering
  byte      retries;    //  number of watering runs before give up (if high not reached)
  byte      watertime;  //  pumping duration
  byte      saturate;   //  saturation duration
  byte      gotosleep;  //  hour to go goodnight (e.g., 22)
  byte      wakeup;     //  hour to wake up (e.g., 08)
  byte      wkendadj;   //  weekend wake up adjustment time
  int       wl_depth;   //  water container depth (empty), in mm
  int       wl_low;     //  water container level low mark, in mm
  //  12 bytes

  // wifi parameters
  byte      is_ap;      //  is this system configured to be an access point or connect to a network
  char      ssid[32];   //  SSID of the network to connect to -OR- SSID of the AP to create
  char      pwd[65];    //  Password for the network to be connected to -OR- AP password
  char      ssid_ap[32];//  SSID of the network to connect to -OR- SSID of the AP to create
  char      pwd_ap[65]; //  Password for the network to be connected to -OR- AP password
  // 195 bytes

  // power parameters
  bool      powersave;  //  false: run continuosly, true: run in a power saving mode
  // 2 bytes
  // total of 209 bytes
} TParameters;

TParameters parameters;

typedef struct {
  unsigned long magic;      // token: 1234567890 for validation
  unsigned long ccode;      // control code for the magic number = ~magic
  unsigned long ttime;      // unix time at the time of reset
  bool          hasntp;     // was the time ntp driven
  int           sleep_mult; // sleep multiplyer
  // Total of 16 bytes
} TTimeStored;

TTimeStored    time_restore;
#define       MAGIC_NUMBER  1234567890UL
#define       WAKE_DELAY    1            // 3 seconds to wake up 

typedef struct {
  time_t    water_start;  // time watering run started
  byte      hum_start;    // humidity at the start
  int       wl_start;     // water level when started, in mm
  byte      num_runs;     // number of runs it took to water
  byte      run_duration; // run durations
  time_t    water_end;    // time watering stopped
  byte      hum_end;      // humidity at the end of run
  int       wl_end;       // water level when ended, in mm
} TWaterLog;

TWaterLog water_log;

// Support for timezones:
//US Eastern Time Zone (New York)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);


// NTP Related Definitions
#define NTP_PACKET_SIZE  48 // NTP time stamp is in the first 48 bytes of the message

/* Don't hardwire the IP address or we won't get the benefits of the pool.
    Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
//unsigned int localNtpPort = 2390;      // local port to listen for UDP packets
#define LOCAL_NTP_PORT  2390
#define LOCAL_HTTP_PORT 80
#define LOCAL_DNS_PORT  53

IPAddress apIP(192, 168, 1, 1);

ESP8266WebServer server(LOCAL_HTTP_PORT);          // local webserver
DNSServer dnsServer;

RTC_Millis rtc;

time_t        bootTime;           // Timestamp of the device start up
time_t        tickTime;           // Timestamp of the current tick
unsigned long epoch;              // Time reported by NTP
bool          nightMode = false;  // Determine whether it is night once during tick, and then use the variable instead.

// Execution
bool connectedToAP = false; // if connected to AP, presumably you can query and set the time
// and properly destinguish between night and day.
// Otherwise, the monitoring needs to run once a day
// from the time the device was started assuming that happens
// during the day
bool hasNtp;

// WiFi specific
WiFiEventHandler disconnectedEventHandler;
WiFiEventHandler clientConnectedEventHandler;
WiFiEventHandler clientDisconnectedEventHandler;
unsigned int numClients;

// Ultrasonic measurment related
volatile bool pulseBusy = false;
volatile bool pulseTimeout = false;
volatile unsigned long pulseStart = 0;
volatile unsigned long pulseStop = 0;
volatile bool clockStarted = false;

// Code
// ------------------------------------------------------------------

#ifdef _TEST_
void testTicker() {
  long i = tTest.getRunCounter();

  if ( tWater.isEnabled() ) {
    currentHumidity += 5;
    if ( tWater.getRunCounter() & 1 ) currentWaterLevel -= 4;
  }
  else {
    currentHumidity -= 1;
  }

  if ( currentHumidity < 20 ) currentHumidity = 20;
  if ( currentHumidity > 100 ) currentHumidity = 100;

  if ( currentWaterLevel < 0 ) currentWaterLevel = 0;

  Serial.println();
  Serial.println(F("TEST MODE"));
  Serial.println(F("========="));
  Serial.print(F("Current time: ")); printTime(now()); Serial.println();
  Serial.print(F("Current humidity   : ")); Serial.print(currentHumidity); Serial.println("%");
  Serial.print(F("Current water level: ")); Serial.print(currentWaterLevel); Serial.println(" mm");
  Serial.print(F("Status: "));
  if (tWater.isEnabled() ) {
    if (tWater.getRunCounter() & 1) Serial.println(F("watering..."));
    else Serial.println(F("saturating..."));
  }
  else {
    Serial.println(F("idle."));
  }
}
#endif

// Utility methods
// ---------------

//#define SR04_TRIGGER_PIN  14  // D5 nomemcu pin
//#define SR04_ECHO_PIN     12  // D6 nomemcu pin

void ping(unsigned long aTimeout) {  // in millis

#ifdef _TEST_
  pulseComplete.signal();
  return;
#endif

  pulseBusy = true;
  pulseTimeout = false;
  digitalWrite(SR04_TRIGGER_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(SR04_TRIGGER_PIN, HIGH);
  tPing.setInterval (aTimeout);
  pulseStart = 0;
  clockStarted = false;
  delayMicroseconds(10);
  digitalWrite(SR04_TRIGGER_PIN, LOW);
  //  pingDistance = pulseIn(SR04_ECHO_PIN, HIGH) * 170L / 10000L;
  attachInterrupt(SR04_ECHO_PIN, &pingStartStopClock, CHANGE);
  tPing.restartDelayed();
}

void pingStartStopClock() {
  if (clockStarted) {
    pulseStop = micros();
    detachInterrupt(SR04_ECHO_PIN);
    pulseBusy = false;
    tPing.disable();
    pulseComplete.signal();
  }
  else {
    pulseStart = micros();
    //    detachInterrupt(SR04_ECHO_PIN);
    //    attachInterrupt(SR04_ECHO_PIN, &pingStartStopClock, FALLING);
    clockStarted = true;
    tPing.restartDelayed();
  }
}

void pingTOut() {
  if (pulseBusy) {
    clockStarted = true;
    pingStartStopClock();
  }
  pulseTimeout = true;
}

#define WL_SAMPLES  5
long  wlData[WL_SAMPLES];
avgFilter wl(WL_SAMPLES, wlData);

void waterLevelCalc() {
  long cwl = 0;

#ifdef _DEBUG_
  long d, mm;
#endif

#ifdef _TEST_
  return;
#endif

  if (pulseTimeout) cwl = 0;
  else {
    // 343 m/s = 343000 mm/s = 0.343 mm/us. double distance = 0.1715 mm/us
    cwl = parameters.wl_depth - (pulseStop - pulseStart) * US_TO_MM_FACTOR / 1000;

#ifdef _DEBUG_
    d = pulseStop - pulseStart;
    mm = d * US_TO_MM_FACTOR / 1000;
#endif

    if ( cwl < 0 ) cwl = 0;
    if ( cwl > parameters.wl_depth ) cwl = parameters.wl_depth;
  }
  currentWaterLevel = wl.value(cwl);

#ifdef _DEBUG_
  //  Serial.print(millis());
  //  Serial.print(F(": waterLevelCalc. d = "));
  //  Serial.println(d);
  //  Serial.print(F("mm = "));
  //  Serial.println(mm);
  //  Serial.print(F("cwl = "));
  //  Serial.println(currentWaterLevel);
#endif
}


// Motor
void motorState(bool aState) {
  digitalWrite(PUMP_PWR_PIN, aState ? HIGH : LOW);
}

void motorOn() {
  motorState(true);
}

void motorOff() {
  motorState(false);
}

// Probes
//#define MOISTURE_PWR_PIN  13  // D7
//#define MOISTURE_GND_PIN  15  // D8
void probePowerOn() {
  digitalWrite(MOISTURE_GND_PIN, LOW);
  digitalWrite(MOISTURE_PWR_PIN, HIGH);
}

void probePowerReverse() {
  digitalWrite(MOISTURE_PWR_PIN, LOW);
  digitalWrite(MOISTURE_GND_PIN, HIGH);
}

void probePowerOff() {
  digitalWrite(MOISTURE_PWR_PIN, LOW);
  digitalWrite(MOISTURE_GND_PIN, LOW);
}

bool hasLeaked() {
  bool r;
  pinMode(LEAK_PIN, INPUT_PULLUP);
  delay(5);

  r = digitalRead(LEAK_PIN);

  pinMode(LEAK_PIN, OUTPUT);
  digitalWrite(LEAK_PIN, LOW);

  return (r == LOW);
}

bool canWater() {
  return !( hasLeaked() || currentWaterLevel < parameters.wl_low || isNight() );
}

bool isNight() {

  nightMode = false;
  if ( hasNtp ) {

    time_t tnow = myTZ.toLocal( now() );

    int hr = hour(tnow);
    int wkp = parameters.wakeup;

#ifdef _DEBUG_
    Serial.print(millis());
    Serial.print(F(": isNight. hr="));
    Serial.println(hr);
    //  return false;
#endif

    //  Add adjusting hours to the wakeup time for Saturday and Sunday
    if ( weekday(tnow) == dowSunday || weekday(tnow) == dowSaturday ) wkp += parameters.wkendadj;
    nightMode = ( hr >= parameters.gotosleep || hr < wkp );

#ifdef _DEBUG_
    Serial.print(F("parameters.gotosleep = ")); Serial.println(parameters.gotosleep);
    Serial.print(F("wkp = ")); Serial.println(wkp);
    Serial.print(F("nightMode = ")); Serial.println(nightMode);
    //  return false;
#endif

  }
  return nightMode;
}

// 3 Color LED
int rgbRed = 0, rgbGreen = 0, rgbBlue = 0;
bool pwmRed = false, pwmGreen = false, pwmBlue = false;
void led(int aR = -1, int aG = -1, int aB = -1) {
  if (aR >= 0) rgbRed = aR;
  if (aG >= 0) rgbGreen = aG;
  if (aB >= 0) rgbBlue = aB;

  if (rgbRed < 0) rgbRed = 0; if (rgbRed > LEDON) rgbRed = LEDON;
  if (rgbGreen < 0) rgbGreen = 0; if (rgbGreen > LEDON) rgbGreen = LEDON;
  if (rgbBlue < 0) rgbBlue = 0; if (rgbBlue > LEDON) rgbBlue = LEDON;

  if ( nightMode ) {
    analogWrite(RGBLED_RED_PIN, 0);
    analogWrite(RGBLED_GREEN_PIN, 0);
    analogWrite(RGBLED_BLUE_PIN, 0);
  }
  else {
    analogWrite(RGBLED_RED_PIN, rgbRed);
    analogWrite(RGBLED_GREEN_PIN, rgbGreen);
    analogWrite(RGBLED_BLUE_PIN, rgbBlue);
  }
}

void ledOff() {
  led(0, 0, 0);
}


// Parameters
void loadParameters() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": loadParameters."));
#endif
  // Let's see if we have the defaults stored already
  // First lets read the token.

  EEPROM.get(0, parameters);
  if (strcmp(CToken, parameters.token) != 0) {
    // Write down token and defaults
    strncpy(parameters.token, CToken, 7);

    parameters.high = (byte) STOPWATER;
    parameters.low = (byte) NEEDWATER;
    parameters.retries = (byte) RETRIES;
    parameters.watertime = (byte) WATERTIME;
    parameters.saturate = (byte) SATURATE;
    parameters.gotosleep = (byte) GOTOSLEEP;
    parameters.wakeup = (byte) WAKEUP;
    parameters.wkendadj = (byte) WEEKENDADJ;
    parameters.wl_depth = (int) WLDEPTH; //230;
    parameters.wl_low = (int) WLLOW; //210;

    parameters.is_ap = 0; // not a server
    //    parameters.is_ap = 1; // a server
    strncpy(parameters.ssid, CSsid, 32);
    strncpy(parameters.pwd, CPwd, 65);
    ssid = CDSsid + String( ESP.getChipId() );
    strncpy(parameters.ssid_ap, ssid.c_str(), 32);
    strncpy(parameters.pwd_ap, CDPwd, 65);

    parameters.powersave = false;

    saveParameters();
  }
}

void saveParameters() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": saveParameters."));
  Serial.print(F("Token: ")); Serial.println(parameters.token);
  Serial.print(F("High : ")); Serial.println(parameters.high);
  Serial.print(F("Low  : ")); Serial.println(parameters.low);
  Serial.print(F("Retries: ")); Serial.println(parameters.retries);
  Serial.print(F("WaterTm: ")); Serial.println(parameters.watertime);
  Serial.print(F("Saturate: ")); Serial.println(parameters.saturate);
  Serial.print(F("Gotosleep: ")); Serial.println(parameters.gotosleep);
  Serial.print(F("Wakeup: ")); Serial.println(parameters.wakeup);
  Serial.print(F("WkendAdj: ")); Serial.println(parameters.wkendadj);
  Serial.print(F("WL Depth: ")); Serial.println(parameters.wl_depth);
  Serial.print(F("WL Low: ")); Serial.println(parameters.wl_low);
  Serial.print(F("SSID: ")); Serial.println(parameters.ssid);
  Serial.print(F("PWD: ")); Serial.println(parameters.pwd);
  Serial.print(F("AP SSID: ")); Serial.println(parameters.ssid_ap);
  Serial.print(F("AP PWD: ")); Serial.println(parameters.pwd_ap);
  Serial.print(F("PWRSave: ")); Serial.println(parameters.powersave);
#endif

  EEPROM.put(0, parameters);
  EEPROM.commit();
}


// Configuration
void cfgInit() {  // Initiate connection

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": cfgInit."));
#endif

  //  loadParameters();
  ssid = parameters.ssid;
  pwd = parameters.pwd;

#ifdef _DEBUG_
  Serial.println(F("WiFi parameters: "));
  Serial.print(F("SSID: ")); Serial.println(ssid);
  Serial.print(F("PWD : ")); Serial.println(pwd);

  Serial.println(F("AP parameters: "));
  Serial.print(F("SSID: ")); Serial.println(parameters.ssid_ap);
  Serial.print(F("PWD : ")); Serial.println(parameters.pwd_ap);
#endif

  if (parameters.is_ap == 0) { // If not explicitly requested to be an Access Point
    // attempt to connect to an exiting AP
    WiFi.mode(WIFI_STA);
    WiFi.persistent (false);
    WiFi.hostname(CHost);
    WiFi.begin(ssid.c_str(), pwd.c_str());

    // flash led green
    rgbRed = 0; rgbGreen = LEDON; rgbBlue = 0;
    tLedBlink.set(CONNECT_BLINK, TASK_FOREVER, &cfgLed, NULL, &ledOnDisable);
    tLedBlink.enable();

    // check connection periodically
    tConfigure.set(CONNECT_INTRVL, TASK_FOREVER, &cfgChkConnect);
    tConfigure.enableDelayed();

    // set 10 second timeout
    tTimeout.set(CONNECT_TIMEOUT * TASK_SECOND, TASK_ONCE, &connectTOut);
    tTimeout.enableDelayed();
  }
  else {
    ssid = parameters.ssid_ap;
    pwd = parameters.pwd_ap;
    tConfigure.yield(&cfgSetAP);
  }
}

void cfgChkConnect() {  // Wait for connection to AP

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": cfgChkConnect."));
#endif

  if (WiFi.status() == WL_CONNECTED) {
    //    tLedBlink.disable();
    //    tTimeout.disable();

    disconnectedEventHandler = WiFi.onStationModeDisconnected(&onDisconnected);
    connectedToAP = true;

#ifdef _DEBUG_
    Serial.print(F("Connected to AP. Local ip: "));
    Serial.println(WiFi.localIP());
#endif

    // request NTP update
    //    hasNtp = false;

    // flash led green rapidly
    rgbRed = 0; rgbGreen = LEDON; rgbBlue = 0;
    tLedBlink.set(CONNECT_BLINK / 4, TASK_FOREVER, &cfgLed, NULL, &ledOnDisable);
    tLedBlink.enable();

    // check connection periodically
    tConfigure.set(CONNECT_INTRVL, TASK_FOREVER, &cfgChkNTP);
    tConfigure.restartDelayed();

    // set timeout
    tTimeout.set(NTP_TIMEOUT * TASK_SECOND, TASK_ONCE, &ntpTOut);
    tTimeout.enableDelayed();

    // Initiate NTP updates
    //    tNtpUpdater.set(SECS_PER_DAY * TASK_SECOND, TASK_ONCE, &ntpUpdate);
    //    tNtpUpdater.enableDelayed();
    tNtpUpdater.restart();

    //    tConfigure.yield(&cfgChkNTP);
  }
  else {
    if (tConfigure.getRunCounter() % 5 == 0) {

#ifdef _DEBUG_
      Serial.println(F("Re-requesting connection to AP..."));
#endif

      //      wifi_set_phy_mode(PHY_MODE_11G);
      //      WiFi.setOutputPower(20.5);
      //      WiFi.setAutoConnect(false);
      WiFi.disconnect(true);
      yield();
      WiFi.hostname(CHost);
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid.c_str(), pwd.c_str());
    }
  }
}

void doNtpUpdateInit() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": doNtpUpdateInit."));
#endif

  // request NTP update
  udp.begin(LOCAL_NTP_PORT);
  if ( WiFi.hostByName(ntpServerName, timeServerIP) ) { //get a random server from the pool

#ifdef _DEBUG_
    Serial.print(F("timeServerIP = "));
    Serial.println(timeServerIP);
#endif

    sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": sendNTPpacket."));
#endif

  //  Serial.println(F("sending NTP packet..."));
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
  //  delay(100);
}

void cfgChkNTP() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": cfgChkNTP."));
#endif

  if ( tConfigure.isFirstIteration() || tConfigure.getRunCounter() % 5 == 0 ) {
    udp.stop();
    doNtpUpdateInit();
  }
  else {
    if ( doNtpUpdateCheck()) {
      doSetTime(epoch);

#ifdef _DEBUG_
      Serial.println(F("NTP Update successful"));
#endif

      bootTime = now();
      hasNtp = true;

      tLedBlink.disable();
      tTimeout.disable();
      tConfigure.yield(&cfgFinish);
      udp.stop();
    }
    else {
      delay (100);
    }
  }
}


#ifdef _DEBUG_ || _TEST_
void printTime(DateTime aTime) {
  //    DateTime now = rtc.now();

  //    Serial.print(F("UTC date/time: "));
  Serial.print(aTime.year(), DEC);
  Serial.print('-');
  Serial.print(aTime.month(), DEC);
  Serial.print('-');
  Serial.print(aTime.day(), DEC);
  Serial.print(' ');
  Serial.print(aTime.hour(), DEC);
  Serial.print(':');
  Serial.print(aTime.minute(), DEC);
  Serial.print(':');
  Serial.print(aTime.second(), DEC);
  //    Serial.println();

  //    Serial.print(F(" seconds since 1970: "));
  //    Serial.println(now.unixtime());

}
#endif

void doSetTime(unsigned long aTime) {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": doSetTime."));
#endif

  //  rtc.begin();
  rtc.adjust( DateTime(aTime) );
  setTime( rtc.now().unixtime() );

#ifdef _DEBUG_
  //  delay(1000);
  printTime(now());
  Serial.println();
#endif
}


bool doNtpUpdateCheck() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": doNtpUpdateCheck."));
#endif

  int cb = udp.parsePacket();
  if (cb) {
#ifdef _DEBUG_
    Serial.print(F("Packet received, length="));
    Serial.println(cb);
#endif

    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    // now convert NTP time into everyday time:
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;

#ifdef _DEBUG_
    // print Unix time:
    Serial.print(F("Unix time = "));
    Serial.println(epoch);
#endif
    return (epoch != 0);
  }
  return false;
}

time_t getTime() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": getTime."));
#endif

  return rtc.now().unixtime();
}

void ntpUpdate() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": ntpUpdate."));
#endif

  if ( tNtpUpdater.isFirstIteration() || tNtpUpdater.getRunCounter() % 5 == 0) {
    udp.stop();
    doNtpUpdateInit();
  }
  if ( doNtpUpdateCheck() ) {
    if ( !hasNtp ) tTicker.restart();
    hasNtp = true;
    //    rtc.adjust( DateTime(epoch) );
    //    checkUpdateTime();
    doSetTime(epoch);
    tNtpUpdater.disable();
    udp.stop();
#ifdef _DEBUG_
    Serial.println(F("NTP update successful"));
#endif

  }
  else {
    //    delay(100);
  }
}

bool globRet;
void cfgSetAP() {  // Configure server

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": cfgSetAP."));
#endif

  wifi_set_phy_mode(PHY_MODE_11G);
  WiFi.setOutputPower(20.5);
  WiFi.setAutoConnect(false);
  WiFi.disconnect(true);
  yield();
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  if ( pwd.length() == 0 )
    globRet = WiFi.softAP( ssid.c_str());
  else
    globRet = WiFi.softAP( ssid.c_str(), pwd.c_str() );

  dnsServer.setTTL(300);
  dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);
  dnsServer.start(LOCAL_DNS_PORT, CHost, apIP);

  // flash led blue
  rgbRed = LEDON; rgbGreen = 0; rgbBlue = LEDON;
  tLedBlink.set(CONNECT_BLINK, TASK_FOREVER, &cfgLed, NULL, &ledOnDisable);
  tLedBlink.enable();

  // check connection periodically
  tConfigure.set(CONNECT_INTRVL, TASK_FOREVER, &cfgChkAP);
  tConfigure.enableDelayed();

  // set timeout
  tTimeout.set(CONNECT_TIMEOUT * TASK_SECOND, TASK_ONCE, &serverTOut);
  tTimeout.enableDelayed();
}

void cfgChkAP() {  // Wait for AP mode to happen

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": cfgChkAP."));
#endif

  if (globRet) {
    tLedBlink.disable();
    tTimeout.disable();
    tConfigure.yield(&cfgFinish);

    //    epoch = STATIC_TIME;
    //    doSetTime();

#ifdef _DEBUG_
    Serial.print(F("Running as AP. Local ip: "));
    Serial.println(WiFi.softAPIP());
#endif
    clientConnectedEventHandler = WiFi.onSoftAPModeStationConnected(&onClientConnected);
    clientDisconnectedEventHandler = WiFi.onSoftAPModeStationDisconnected(&onClientDisconnected);
  }
}

void cfgFinish() {  // WiFi config successful

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": cfgFinish."));
#endif

  led(rgbRed / 4, rgbGreen / 4, rgbBlue / 4);
  //  MDNS.begin(CHost);
  tTicker.restart();

  SPIFFS.begin();
  server.on("/", serverHandleRoot);
  //  server.on("/confnetworksave", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleConfnetworksave);
  server.on("/confnetworksave", HTTP_POST, handleConfnetworksave);
  server.on("/confwatersave", HTTP_POST, handleConfwatersave);
  server.on("/reset", HTTP_POST, resetDevice);
  server.onNotFound(serverHandleNotFound);


#ifdef _DEBUG_
  // =============== FSBrowse code ====================
  //SERVER INIT
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  //load editor
  server.on("/edit", HTTP_GET, []() {
    if (!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
  });
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, []() {
    server.send(200, "text/plain", "");
  }, handleFileUpload);

  //  //called when the url is not defined here
  //  //use it to load content from SPIFFS
  //  server.onNotFound([](){
  //    if(!handleFileRead(server.uri()))
  //      server.send(404, "text/plain", "FileNotFound");
  //  });

  // ==================================================
#endif


  server.begin();
//  server.setNoDelay(true);  // underlying wifiserver is not exposed

  //  MDNS.addService("http", "tcp", LOCAL_HTTP_PORT);

  if ( connectedToAP ) tHandleClients.restart();

#ifdef _DEBUG_
  Serial.println("HTTP server started");
#endif
}

void ticker() {
#ifdef _DEBUG_ || _TEST_
  Serial.println();
  Serial.print(millis());
  Serial.println(F(": ticker."));
#endif

  tickTime = now();

  if ( parameters.powersave ) {
    tSleepTimeout.restartDelayed();
    sleepTime.setWaiting(2);
    tSleep.waitFor(&sleepTime);
  }

  if ( isNight() ) {

#ifdef _DEBUG_ || _TEST_
    Serial.println(F("Night mode: LED off; SleepTime-SIGNAL"));
#endif

    ledOnDisable();
    sleepTime.signal();
    return;
  }

  if ( !tWater.isEnabled() ) {

    // Launch measurment and watering
    led();
    measureRestart();
  }

  if ( connectedToAP )
    tNtpUpdater.restart();
}

void sleepTimeout() {

#ifdef _DEBUG_ || _TEST_
  Serial.print(millis());
  Serial.println(F(": sleepTimeout."));
#endif

  sleepTime.signal();
}


bool flip = false;
void cfgLed() {
  flip = !flip;

  if (flip) {
    led();
  }
  else {
    ledOnDisable();
  }
}

void ledOnDisable() {
  analogWrite(RGBLED_RED_PIN, 0);
  analogWrite(RGBLED_GREEN_PIN, 0);
  analogWrite(RGBLED_BLUE_PIN, 0);
  flip = false;
}

void connectTOut() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": connectTOut."));
#endif

  connectedToAP = false;
  WiFi.disconnect();
  ssid = CDSsid + String( ESP.getChipId() );
  pwd = CDPwd;

#ifdef _DEBUG_
  Serial.println( F("Setting up AP:") );
  Serial.print(F("SSID: ")); Serial.println(ssid);
  Serial.print(F("PWD : ")); Serial.println(pwd);
  Serial.print(F("Chip ID: ")); Serial.println(ESP.getChipId());
#endif

  tLedBlink.disable();
  tConfigure.yield(&cfgSetAP);  // configure as server
}

void ntpTOut() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": ntpTOut."));
#endif

  tLedBlink.disable();
  rgbRed = LEDON; rgbGreen = 242 * (LEDON / 255); rgbBlue = 0;  // configure as yellow
  //  rgbRed = LEDON/4; rgbGreen = (200 * (LEDON / 255))/4; rgbBlue = 0;  // configure as yellow
  tConfigure.yield(&cfgFinish);  // configure as server
}

void serverTOut() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": serverTOut. Full stop."));
#endif

  ts.disableAll();

  while (1) {
    led(LEDON, 0, 0);
    delay(2000);
    led(0, 0, 0);
    delay(500);
  }
}

void onDisconnected(const WiFiEventStationModeDisconnected& event) {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": onDisconnected."));
#endif

  WiFi.disconnect();
  tConnected.restartDelayed(CONNECT_TIMEOUT * TASK_SECOND);
}


void onClientConnected(const WiFiEventSoftAPModeStationConnected& event) {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": onClientConnected."));
#endif

  numClients++;
  if ( numClients ) tHandleClients.restart();
}

void onClientDisconnected(const WiFiEventSoftAPModeStationDisconnected& event) {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": onClientDisconnected."));
#endif

  if (numClients) {
    if ( --numClients == 0 ) tHandleClients.disable();
  }
}


void connectedChk() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": connectedChk."));
#endif

  if ( tConnected.isFirstIteration() ) {
    WiFi.hostname(CHost);
    WiFi.mode(WIFI_STA);
    WiFi.begin(parameters.ssid, parameters.pwd);
    //    delay(100);
    // set timeout
    tTimeout.set(CONNECT_TIMEOUT * TASK_SECOND, TASK_ONCE, &connectedChkTOut);
    tTimeout.enableDelayed();
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    tTimeout.disable();
    tConnected.disable();
  }
}


void connectedChkTOut () {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": connectedChkTOut."));
#endif

  resetDevice();
}


void sleepCallback() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": sleepCallback."));
#endif
  time_t tnow = now();
  time_t ticker = TICKER / 1000;
  time_t tdesired = ticker + tickTime;

  if ( tdesired > tnow ) {
    saveTimeToRTC( tdesired );

#ifdef _DEBUG_ || _TEST_
    Serial.println(F("Storing TTimeStored structure to RTC memory"));
    Serial.print(F("Magic number:")); Serial.println(time_restore.magic);
    Serial.print(F("ttime number:")); Serial.println(time_restore.ttime);
    Serial.print(F("Tick epoch time:")); Serial.println(tickTime);
    Serial.print(F("Now  epoch time:")); Serial.println(tnow);
    Serial.print(F("ticket interval:")); Serial.println(ticker);
    unsigned long delta = tdesired - tnow;
    Serial.print(F("sleep interval:")); Serial.println(delta);

    Serial.flush();
    delay(10);
    Serial.end();
#endif

    // !-------------------!-------------------------------------------!
    // tick                tnow                                       tick+interval
    // <--- tnow - tick --> <               tick+interval
    time_restore.sleep_mult = 0; // no additional sleep
    ts.disableAll();
    WiFi.disconnect();
    SPIFFS.end();
    ESP.deepSleep( ( tdesired - tnow ) * 1000000UL );
    delay(100);
  }
  else {
    resetDevice();
  }
}

void saveTimeToRTC (unsigned long aUt) {
  time_restore.magic = MAGIC_NUMBER;
  time_restore.ccode = ~time_restore.magic;
  time_restore.ttime = aUt;
  time_restore.hasntp = hasNtp;
  byte *p = (byte*) &time_restore;
  ESP.rtcUserMemoryWrite( 64, (uint32_t*) p, sizeof(TTimeStored) );
  p = (byte*) &water_log;
  ESP.rtcUserMemoryWrite( 128, (uint32_t*) p, sizeof(TWaterLog) );
  //  ESP.rtcUserMemoryWrite( 0, (uint32_t*) p, sizeof(TTimeStored)/sizeof( uint32_t) );
  //  p = (byte*) &water_log;
  //  ESP.rtcUserMemoryWrite( 256, (uint32_t*) p, sizeof(TWaterLog)/sizeof(uint32_t) );
}


void resetDevice() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": resetDevice."));
#endif


  saveTimeToRTC(now());

#ifdef _DEBUG_ || _TEST_
  Serial.println(F("Storing TTimeStored structure to RTC memory"));
  Serial.print(F("Magic number:")); Serial.println(time_restore.magic);
  Serial.print(F("ttime number:")); Serial.println(time_restore.ttime);
  Serial.flush();
  delay(10);
  Serial.end();
#endif

  ts.disableAll();
  WiFi.disconnect();
  SPIFFS.end();
  ESP.reset(); // If device cannot reconnect to the AP, it is reset to try again and/or become an Access Point
  delay(100);
}

//bool WLReported = false;
bool measureWLOnEnable() {
  //  WLReported = false;
  wl.initialize();
  return true;
}

void measureWL() {

#ifdef _DEBUG_
  //  Serial.print(millis());
  //  Serial.println(F(": measureWL.));
#endif

  pulseComplete.setWaiting();
  tWaterLevel.waitFor(&pulseComplete);
  ping(4);
  if (tMesrLevel.getRunCounter() == (WL_SAMPLES + 1) ) measurementsReady.signal();
}




// Measurment and Watering
#define HUM_SAMPLES  5
long  humData[HUM_SAMPLES];
avgFilter hum(HUM_SAMPLES, humData);

long measureHumidity()
{
  long r, l;

  l = analogRead( MOISTURE_PIN );

#ifdef _DEBUG_
  //  Serial.print(millis());
  //  Serial.print(F(": measureHumidity. l = "));
  //  Serial.println(l);
#endif

#ifdef _TEST_
  return currentHumidity;
#endif

  if (l > SENSOR_OUT_VALUE) return 0;

  r = (1023 - l) * 100L / 1023L;
#ifdef _DEBUG_
  //  Serial.print(F("% = "));
  //  Serial.println(r);
#endif
  return hum.value(r);
}

//bool MSReported = false;
//bool measureMSOnEnable() {
//  hum.initialize();
//  return true;
//}

void measureMSOnDisable() {
  probePowerOff();
  probeIdle.signal();
  tMesrLevel.disable();
}

void measureMS() {

  // One second to "warm up" the probe
  if ( tMesrMoisture.isFirstIteration() ) {
    probePowerOn();
    probeIdle.setWaiting();
    tMeasureRestart.waitFor(&probeIdle);

    hum.initialize();
    tMesrMoisture.setInterval(TASK_SECOND / 5);
    tMesrMoisture.delay(TASK_SECOND);
    return;
  }

  // then 5 measuments every 200 ms to collect an average of 5
  if ( tMesrMoisture.getRunCounter() <= ( HUM_SAMPLES + 1) )
    currentHumidity = measureHumidity();

  // on the 6th iteration signal that measurement is ready and reverese the current for 2 seconds
  if ( tMesrMoisture.getRunCounter() == ( HUM_SAMPLES + 1 ) ) {
    tMesrMoisture.setInterval(TASK_SECOND + HUM_SAMPLES * 200);
    probePowerReverse();
    measurementsReady.signal();
  }

  // after 2 seconds of reverse current - shut down
  // the total measument time is 4 seconds
  if ( tMesrMoisture.getRunCounter() > ( HUM_SAMPLES + 1 ) ) {
    tMesrMoisture.disable();
  }
}


void measureCallback() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": measureCallback."));
  Serial.print(F("canWater = "));
  bool a = canWater();
  if (a) Serial.println(F("yes")); else Serial.println(F("no"));
  Serial.print(F("currentHumidity = "));
  Serial.println(currentHumidity);
  Serial.print(F("currentWaterLevel = "));
  Serial.println(currentWaterLevel);
  a = tWater.isEnabled();
  Serial.print(F("Watering is "));
  if (a) Serial.println(F("enabled")); else Serial.println(F("disabled"));
#endif

  // check if we can water now and not doing it already
  if (canWater() && currentHumidity > 0 && currentHumidity < parameters.low) {
    if (!tWater.isEnabled()) {
      tWater.setInterval( parameters.watertime * TASK_MINUTE );
      tWater.setIterations( parameters.retries * 2 );
      tWater.restart();
    }
    return;
  }

  // check if we are watering (and maybe should stop)
  if ( tWater.isEnabled() ) {
    if ( !canWater() || currentHumidity >= parameters.high ) {
      tWater.disable();
    }
  }
  else {
    tMeasureRestart.disable();
    ledOff();
    sleepTime.signal();
  }
}

void measureRestart() {
  measurementsReady.setWaiting(2);
  tMeasure.waitFor(&measurementsReady);

  tMesrLevel.restart();
  tMesrMoisture.restart();
}

void waterCallback() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": waterCallback."));
#endif

  if (tWater.getRunCounter() & 1) {
    // during odd runs the pump needs to be turned on, and an even run configured to perform saturation
    motorOn();
    water_log.num_runs++;
    tWater.setInterval( parameters.watertime * TASK_SECOND );

    rgbRed = 0; rgbGreen = 0; rgbBlue = LEDON;
    tLedBlink.set(50, TASK_FOREVER, &cfgLed, NULL, &ledOnDisable);
    tLedBlink.enable();
  }
  else { // even run
    motorOff();
    tWater.setInterval( parameters.saturate * TASK_MINUTE);

    ledOff();
    tLedBlink.set(50, TASK_FOREVER, &ledSaturate, NULL, &ledOnDisable);
    tLedBlink.enable();
  }
}

int delta = 20;
void ledSaturate() {
  rgbBlue += delta;
  led();
  if ( rgbBlue >= LEDON || rgbBlue <= 0 ) delta = -delta;
}



bool waterOnEnable() {

  if ( !canWater() ) {
    motorOff();
    return false;
  }

  water_log.water_start = now();
  water_log.hum_start = currentHumidity;
  water_log.num_runs = 0;
  water_log.run_duration = parameters.watertime;
  water_log.wl_start = currentWaterLevel;

  water_log.water_end = 0;
  water_log.hum_end = 0;
  water_log.wl_end = 0;

  return true;
}

void waterOnDisable() {
  motorOff();

  water_log.water_end = now();
  water_log.hum_end = currentHumidity;
  water_log.wl_end = currentWaterLevel;

  tMeasure.disable();
  tMeasureRestart.disable();
  tLedBlink.disable();
  sleepTime.signal();
}


// Server code
// ===========
void handleClientCallback() {

#ifdef _DEBUG_
  //      Serial.print(millis());
  //      Serial.println(F(": handleClientCallback."));
#endif

  if ( !connectedToAP ) dnsServer.processNextRequest();
  server.handleClient();
  if ( server.client() ) {
    tHandleClients.forceNextIteration();
    tSleepTimeout.restartDelayed();
  }
  else {
    tHandleClients.delay();
  }
}
int indexParser (int aTag, char* buf, int len);
void parseFile( char* aFileName, int (*aParser)(int aTag, char* aBuffer, int aSize) );

void serverHandleRoot() {

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": serverHandleRoot."));
  //
  //  SPIFFS.begin();
  //  {
  //    Dir dir = SPIFFS.openDir("/");
  //    while (dir.next()) {
  //      String fileName = dir.fileName();
  //      size_t fileSize = dir.fileSize();
  //      Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), String(fileSize).c_str());
  //    }
  //    Serial.printf("\n");
  //  }
#endif
  parseFile( "/index.htm", &indexParser);
}

const char* CReplaceTag = "<!-- ###"; //
const int   CRLen = 8;

//#define BUFFER_SIZE   512
#define BUFFER_SIZE   1024

void parseFile( char* aFileName, int(*aParser)(int aTag, char* aBuffer, int aSize) ) {

  if (SPIFFS.exists(aFileName)) {
    char buf[BUFFER_SIZE + 2];
    int len;

    File file = SPIFFS.open(aFileName, "r");

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");

    int skipNext = 0;
    while (file.available()) {

      if ( ( len = file.readBytesUntil('\n', buf, BUFFER_SIZE) ) ) {
        buf[len] = 0;

        if ( skipNext > 0 ) {
          skipNext--;
          continue;
        }

#ifdef _DEBUG_
        //        Serial.print(F("Current html line: "));
        //        Serial.print(len);
        //        Serial.print(" : ");
        //        Serial.println(buf);
#endif

        // Tag is: |<!-- ###0001-blah blah blah|
        //          0123456789012345
        if ( strncmp(CReplaceTag, buf, CRLen) == 0 ) {

          //#ifdef _DEBUG_
          //          Serial.println(F("Found the '<!-- ###' tag."));
          //          Serial.print(F("Current html line: "));
          //          Serial.print(len);
          //          Serial.print(" : ");
          //          Serial.println(buf);
          //#endif
          buf[CRLen + 4] = 0;
          int tag = String(&buf[CRLen]).toInt();

#ifdef _DEBUG_
          //          Serial.print(F("Extracted tag = "));
          //          Serial.println(tag);
#endif
          skipNext = (*aParser)(tag, buf, BUFFER_SIZE);

#ifdef _DEBUG_
          //          Serial.print(F("Parser = "));
          //          Serial.println(buf);
          //          Serial.print(F("skipNext = "));
          //          Serial.println(skipNext);
#endif
        }
        server.sendContent(buf);
      }
    }
    file.close();
  }
}


int indexParser (int aTag, char* buf, int len) {
  time_t tnow;

  switch (aTag) {
    case 1: //0001-DATETIME
      tnow = myTZ.toLocal( now() );
      snprintf( buf, len, "%04d-%02d-%02d %02d:%02d:%02d", year(tnow), month(tnow), day(tnow), hour(tnow), minute(tnow), second(tnow) );
      break;

    case 2: //0002-SYSTEMSTATUS
      if ( tWater.isEnabled() ) {
        if ( tWater.getRunCounter() & 1) {
          snprintf( buf, len, "Watering");
        }
        else {
          snprintf( buf, len, "Saturating");
        }
      }
      else {
        snprintf( buf, len, "Idle");
      }
      break;

    case 3: //0003-SOILHUMIDITY
      snprintf( buf, len, "%d%", currentHumidity);
      break;

    case 4: //0004-LEAKSTATUS
      if ( hasLeaked() ) {
        snprintf( buf, len, "Yes");
      }
      else {
        snprintf( buf, len, "No");
      }
      break;

    case 5: //0005-WATERLEVEL
      snprintf( buf, len, "%d mm", currentWaterLevel);
      break;

    case 14: //0014-LWSTARTTIME
      if ( water_log.water_start ) {
        tnow = myTZ.toLocal( water_log.water_start );
        snprintf( buf, len, "%04d-%02d-%02d %02d:%02d:%02d", year(tnow), month(tnow), day(tnow), hour(tnow), minute(tnow), second(tnow) );
      }
      else
        buf[0] = 0;
      break;

    case 15: //0015-LWSTARTHUM
      if ( water_log.water_start ) {
        snprintf( buf, len, "%d%", water_log.hum_start);
      }
      else
        buf[0] = 0;
      break;

    case 16: //0016-LWSTARTLVL
      if ( water_log.water_start ) {
        snprintf( buf, len, "%d mm", water_log.wl_start);
      }
      else
        buf[0] = 0;
      break;

    case 17: //0017-LWRUNS
      if ( water_log.water_end ) {
        snprintf( buf, len, "%d", water_log.num_runs);
      }
      else
        buf[0] = 0;
      break;

    case 18: //0018-LWENDTIME
      if ( water_log.water_end ) {
        tnow = myTZ.toLocal( water_log.water_end );
        snprintf( buf, len, "%04d-%02d-%02d %02d:%02d:%02d", year(tnow), month(tnow), day(tnow), hour(tnow), minute(tnow), second(tnow) );
      }
      else
        buf[0] = 0;
      break;

    case 19: //0019-LWENDHUM
      if ( water_log.water_end ) {
        snprintf( buf, len, "%d%", water_log.hum_end);
      }
      else
        buf[0] = 0;
      break;

    case 20: //0020-LWENDLVL
      if ( water_log.water_end ) {
        snprintf( buf, len, "%d mm", water_log.wl_end);
      }
      else
        buf[0] = 0;
      break;

    case 21: //0021-RUNAS
      if ( connectedToAP )
        snprintf( buf, len, "Wireless Client. ");
      else
        snprintf( buf, len, "Access Point. ");
      break;

    case 22: //0022-LOCALIP
      if ( connectedToAP )
        snprintf( buf, len, "%s", WiFi.localIP().toString().c_str() );
      else
        snprintf( buf, len, "%s", WiFi.softAPIP().toString().c_str() );
      break;
      break;

    default:
      buf[0] = 0;
      return 0;
  }
  return 1;
}

const char *CHtmlInputText = "<input type=\"text\" value=\"%d\"";
int waterParser (int aTag, char* buf, int len) {
  time_t tnow;

  switch (aTag) {
    case 1: //0001-DATETIME
      tnow = myTZ.toLocal( now() );
      snprintf( buf, len, "%04d-%02d-%02d %02d:%02d:%02d", year(tnow), month(tnow), day(tnow), hour(tnow), minute(tnow), second(tnow) );
      break;

    case 6: //0006-NEEDWATER
      snprintf( buf, len, CHtmlInputText, parameters.low);
      break;

    case 7: //0007-WATERTIME
      snprintf( buf, len, CHtmlInputText, parameters.watertime);
      break;

    case 8: //0008-STOPWTER
      snprintf( buf, len, CHtmlInputText, parameters.high);
      break;

    case 9: //0009-SATURATE
      snprintf( buf, len, CHtmlInputText, parameters.saturate);
      break;

    case 10: //0010-RETRIES
      snprintf( buf, len, CHtmlInputText, parameters.retries);
      break;

    case 11: //0011-GOTOSLEEP
      snprintf( buf, len, CHtmlInputText, parameters.gotosleep);
      break;

    case 12: //0012-WAKEUP
      snprintf( buf, len, CHtmlInputText, parameters.wakeup);
      break;

    case 13: //0013-WEEKENDADJ
      snprintf( buf, len, CHtmlInputText, parameters.wkendadj);
      break;

    case 20: //0020-WLDEPTH
      snprintf( buf, len, CHtmlInputText, parameters.wl_depth);
      break;

    case 21: //0021-MINLEVEL
      snprintf( buf, len, CHtmlInputText, parameters.wl_low);
      break;

    default:
      buf[0] = 0;
      return 0;
  }
  return 1;
}

int networkParser (int aTag, char* buf, int len) {
  time_t tnow;
  String s;

  switch (aTag) {
    case 1: //0001-DATETIME
      tnow = myTZ.toLocal( now() );
      snprintf( buf, len, "%04d-%02d-%02d %02d:%02d:%02d", year(tnow), month(tnow), day(tnow), hour(tnow), minute(tnow), second(tnow) );
      break;

    case 2: //0002-CONFIG
      //    <option value="apoint">Access Point</option><option value="client" selected>Wireless Network</option>
      s = "<option value=\"apoint\"";
      if ( parameters.is_ap ) s += " selected";
      s += ">Access Point</option><option value=\"client\"";
      if ( !parameters.is_ap ) s += " selected";
      s += ">Wireless Network</option>";
      snprintf( buf, len, "%s", s.c_str());
      break;

    case 3: //0003-POWER
      //    <option value="on">Always On</option><option value="save" selected>Power Saving</option>
      s = "<option value=\"on\"";
      if ( !parameters.powersave ) s += " selected";
      s += ">Always On</option><option value=\"save\"";
      if ( parameters.powersave ) s += " selected";
      s += ">Power Saving</option>";
      snprintf( buf, len, "%s", s.c_str());
      break;

    case 4: //0004-SSID
      //    <input type="text" value="your wifi network ssid"
      snprintf( buf, len, "<input type=\"text\" value=\"%s\"", parameters.ssid);
      break;

    case 5: //0005-PWD
      //    <input type="text" value="your wifi password"
      snprintf( buf, len, "<input type=\"text\" value=\"%s\"", parameters.pwd);
      break;

#ifdef _DEBUG_
    case 6: //0006-INFO
      s = ESP.getResetInfo();
      snprintf( buf, len, "%s\n magic no=%lu, unixtime=%lu\n", s.c_str(), time_restore.magic, time_restore.ttime);
      break;
#endif

    case 7: //0004-SSID
      //    <input type="text" value="your wifi network ssid"
      snprintf( buf, len, "<input type=\"text\" value=\"%s\"", parameters.ssid_ap);
      break;

    case 8: //0005-PWD
      //    <input type="text" value="your wifi password"
      snprintf( buf, len, "<input type=\"text\" value=\"%s\"", parameters.pwd_ap);
      break;

    default:
      buf[0] = 0;
      return 0;
  }
  return 1;
}


void serverHandleNotFound() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": serverHandleNotFound."));
  Serial.print(F("Server URI: ")); Serial.println(server.uri());
#endif

  if ( server.uri() == "/index.htm" )   parseFile( "/index.htm", &indexParser);
  else if ( server.uri() == "/confwater.htm" )   parseFile( "/confwater.htm", &waterParser);
  else if ( server.uri() == "/confnetwork.htm" ) parseFile( "/confnetwork.htm", &networkParser);
  else if ( !handleFileRead( server.uri() ) ) {
    String message = "Page Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++) {
      message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", message);
  }
}

bool handleFileRead(String path) {

  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";

  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) {
    if (SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();

    if ( path == "/restart.jpg" )  pageLoaded.signalComplete();
    return true;
  }
  return false;
}

void cpParams( byte* s, byte* d ) {
  for (int i = 0; i < sizeof(TParameters); i++, s++, d++) {
    *d = *s;
  }
}

const char* CSaved = "<html><head></head><body><script>window.alert(\"Parameters saved\");window.location.href='/index.htm';</script></body></html>";
const char* CRestart = "<html><head></head><body><script>window.alert(\"Parameters saved\");window.location.href='/confrestart.htm';</script></body></html>";
void handleConfnetworksave() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": handleConfnetworksave."));
  Serial.println(server.uri());
  Serial.print("Server args = "); Serial.println(server.args());
  for (int i = 0; i < server.args(); i++) {
    Serial.print(server.argName(i)); Serial.print(" = "); Serial.println(server.arg(i));
  }
#endif

  if ( server.args() == 7 && server.uri() == "/confnetworksave" ) {

    pageLoaded.setWaiting();
    tReset.waitFor(&pageLoaded);

    TParameters temp;
    cpParams( (byte*) &parameters, (byte*) &temp );

    temp.is_ap = ( server.arg("config") == "apoint" );
    temp.powersave = ( server.arg("power") == "save" );
    strncpy(temp.ssid, server.arg("ssid").c_str(), 32);
    strncpy(temp.pwd, server.arg("pwd").c_str(), 65);
    strncpy(temp.ssid_ap, server.arg("ssidap").c_str(), 32);
    strncpy(temp.pwd_ap, server.arg("pwdap").c_str(), 65);

    if ( true ) { // Replace with real validation
      cpParams( (byte*) &temp, (byte*) &parameters );
      saveParameters();
    }

    server.send(200, "text/html", CRestart);
  }
  else {
    return server.send(500, "text/plain", F("Invalid Arguments"));
  }
}


void handleConfwatersave() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": handleConfwatersave."));
  Serial.println(server.uri());
  Serial.print("Server args = "); Serial.println(server.args());
  for (int i = 0; i < server.args(); i++) {
    Serial.print(server.argName(i)); Serial.print(" = "); Serial.println(server.arg(i));
  }
#endif

  TParameters temp;
  cpParams( (byte*) &parameters, (byte*) &temp );

  if ( server.args() == 11 && server.uri() == "/confwatersave" ) {

    temp.low = server.arg("needwater").toInt();
    temp.watertime = server.arg("watertime").toInt();
    temp.high = server.arg("stopwater").toInt();
    temp.saturate = server.arg("saturate").toInt();
    temp.retries = server.arg("retries").toInt();
    temp.gotosleep = server.arg("gotosleep").toInt();
    temp.wakeup = server.arg("wakeup").toInt();
    temp.wkendadj = server.arg("weekendadj").toInt();
    temp.wl_depth = server.arg("wldepth").toInt();
    temp.wl_low = server.arg("minlevel").toInt();

    // ======================================
    //  BIG TO DO: Parameter validation
    // ======================================

    if ( true ) { // replace with parameter validation result
      cpParams( (byte*) &temp, (byte*) &parameters );
      saveParameters();

      server.send(200, "text/html", CSaved);
    }
    else {
      server.send(500, "text/plain", F("Invalid Parameters"));
    }
  }
  else {
    return server.send(500, "text/plain", F("Invalid Arguments"));
  }
}

String getContentType(String filename) {
  if (server.hasArg("download")) return "application/octet-stream";
  else if (filename.endsWith(".htm")) return "text/html";
  else if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".gif")) return "image/gif";
  else if (filename.endsWith(".jpg")) return "image/jpeg";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".xml")) return "text/xml";
  else if (filename.endsWith(".pdf")) return "application/x-pdf";
  else if (filename.endsWith(".zip")) return "application/x-zip";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}


// Startup code
// ============

void configurePins() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": configurePins."));
#endif

  pinMode(MOISTURE_PIN, INPUT);
  pinMode(MOISTURE_PWR_PIN, OUTPUT); digitalWrite(MOISTURE_PWR_PIN, LOW);
  pinMode(MOISTURE_GND_PIN, OUTPUT); digitalWrite(MOISTURE_GND_PIN, LOW);

  pinMode(RGBLED_RED_PIN, OUTPUT); digitalWrite(RGBLED_RED_PIN, LOW);
  pinMode(RGBLED_GREEN_PIN, OUTPUT); digitalWrite(RGBLED_GREEN_PIN, LOW);
  pinMode(RGBLED_BLUE_PIN, OUTPUT); digitalWrite(RGBLED_BLUE_PIN, LOW);

  pinMode(SR04_ECHO_PIN, INPUT);
  pinMode(SR04_TRIGGER_PIN, OUTPUT); digitalWrite(SR04_TRIGGER_PIN, LOW);

  pinMode(PUMP_PWR_PIN, OUTPUT); digitalWrite(PUMP_PWR_PIN, LOW);
  pinMode(LEAK_PIN, OUTPUT); digitalWrite(LEAK_PIN, LOW);
}


void setup() {
  configurePins();
  rtc.begin();
  rtc.adjust( DateTime(STATIC_TIME) );
  setSyncProvider(&getTime);   // the function to get the time from the RTC

#ifdef _DEBUG_ || _TEST_
  Serial.begin(74880);
  delay(5000);
#endif

  hasNtp = false;
  if ( ESP.getResetReason() != CWakeReasonReset ) {
    byte *p = (byte *) &time_restore;
    //    if ( ESP.rtcUserMemoryRead( 0, (uint32_t*) p, sizeof(TTimeStored)/sizeof(uint32_t) )) {
    if ( ESP.rtcUserMemoryRead( 64, (uint32_t*) p, sizeof(TTimeStored) )) {


#ifdef _DEBUG_ || _TEST_
      Serial.println(F("Read TTimeStored structure from RTC memory"));
      Serial.print(F("Magic number:")); Serial.println(time_restore.magic);
      Serial.print(F("ttime number:")); Serial.println(time_restore.ttime);
      Serial.print(F("has NTP     :")); Serial.println(time_restore.hasntp);
#endif

      if ( time_restore.magic == MAGIC_NUMBER && time_restore.ccode == ~(MAGIC_NUMBER) ) {
        //        rtc.adjust( DateTime( time_restore.ttime + WAKE_DELAY ) );
        doSetTime(time_restore.ttime + WAKE_DELAY);
        hasNtp = time_restore.hasntp;

        p = (byte *) &water_log;
        //        ESP.rtcUserMemoryRead( 256, (uint32_t*) p, sizeof(TWaterLog)/sizeof(uint32_t) );
        ESP.rtcUserMemoryRead( 128, (uint32_t*) p, sizeof(TWaterLog) );

#ifdef _DEBUG_ || _TEST_
        Serial.println(F("Time and Water log restored from RTC memory"));
#endif

      }
    }
  }
  else {

#ifdef _DEBUG_ || _TEST_
    Serial.println(F("Erased config set AutoConnect to false"));
#endif

    ESP.eraseConfig();
    WiFi.setAutoConnect(false);
    WiFi.setAutoReconnect(false);
  }

  checkUpdateTime();
  setSyncInterval(3600);      // Sync time every 1 hour with RTC

  EEPROM.begin(sizeof(parameters) + sizeof(water_log));

  loadParameters();

#ifdef _DEBUG_ || _TEST_
  Serial.println(F("IoT Plant Watering System"));
#endif

  if ( !isNight() ) {
    //    for (int i = 0; i < 3; i++) {
    led(LEDON, 0, 0); delay(500);
    led(0, LEDON, 0); delay(500);
    led(0, 0, LEDON); delay(500);
    led(0, 0, 0); delay(500);
    //    }
  }

#ifdef _TEST_ || _DEBUG_
  led(LEDON, LEDON, LEDON);
  delay(5000);
#endif

  led(0, 0, 0);


  ts.setHighPriorityScheduler(&hpts);
  ts.startNow();
}

void loop() {
  ts.execute();
}

#ifdef _DEBUG_
// ====================== FSBrowse code ========================

File fsUploadFile;
void handleFileList() {
  if (!server.hasArg("dir")) {
    server.send(500, "text/plain", "BAD ARGS");
    return;
  }

  String path = server.arg("dir");
  Serial.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while (dir.next()) {
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }

  output += "]";
  server.send(200, "text/json", output);
}
void handleFileCreate() {
  if (server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.println("handleFileCreate: " + path);
  if (path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if (SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if (file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileDelete() {
  if (server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.println("handleFileDelete: " + path);
  if (path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if (!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileUpload() {
  if (server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    //Serial.print("handleFileUpload Data: "); Serial.println(upload.currentSize);
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile)
      fsUploadFile.close();
    Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
  }
}

#endif
