// *****************************************************
// Firmware per Wifi Caliper
// -----------------------------------------------------
// Processore: Espressif ESP32-S3
// *****************************************************

// se dà errori nell'upload (baud rate a 921600) fare così:
// al successivo upload (nella fase di compilazione) staccare usb, tenere premuto tasto BOOT, attaccare usb (tenendo il BOOT), premere RST e rilasciarlo, rilasciare BOOT

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <ArduinoJson.h>  // Gestisce il formato JSON
#include <Base64.h>
#include "esp_task_wdt.h"
#include <Preferences.h>
Preferences settings;

// PROCESSORE
String chipId;
const uint8_t *bleaddr_u8;  // Identificativo MAC del modulo Bluetooth (array di interi)

#define FWBUILD_DATE 20231114  // yyyymmvvv [y = year, m = month, v = version]
const short FWversion = 1;


/*  GEQO CONFIG --------------------------------------------------------------*/
String geqo_url = "api.geqo.it";  // api.extjsdev.cloud     api.geqo.it
String geqo_dbname = "trackmes";
String geqo_machine = "001";
String geqo_user = "admin";
String geqo_pwd = "trackmes456";
String geqo_api = "test";
String geqo_align = "3600";
String geqo_zone = "Europe/Rome";
String iotName = "";


/*  WEB CONFIG ---------------------------------------------------------------*/
const char *www_username = "admin";
const char *www_password = "1234";

/*  ETH CONFIG ---------------------------------------------------------------*/
//#define EthEnable
#ifdef EthEnable
IPAddress EthIP(0, 0, 0, 0);
IPAddress EthGW(0, 0, 0, 0);
IPAddress EthSM(255, 255, 255, 0);
IPAddress EthDNS(8, 8, 8, 8);
byte mac[6];
#endif

/*  WIFI CONFIG --------------------------------------------------------------*/
#define WifiEnable
#ifdef WifiEnable
#define WifiTimeout 15000;
const char *WifiDefaultSSID = "CALIPER";  // prima SEM
const char *WifiDefaultPassword = "12345678";
#endif

/* GPRS CONFIG ---------------------------------------------------------------*/
//#define GprsEnable
#ifdef GprsEnable
String GPRS_PIN = "";       //1503
String GPRS_APN = "iliad";  //iliad TM mobile.vodafone.it;
String GPRS_LOGIN = "";
String GPRS_PASSWORD = "";
#endif

/* CORE MANAGER --------------------------------------------------------------*/
TaskHandle_t Task1;
TaskHandle_t Task2;

/* OTA MANAGER ---------------------------------------------------------------*/
#define NO_OTA_NETWORK
#include <ArduinoOTA.h>
#include <Wire.h>

/* TIME NTP RTC       --------------------------------------------------------*/
#include <Time.h>
#include "zones.h"
#include <ESP32Time.h>
ESP32Time rtc;
#include "timestamp32bits.h"
timestamp32bits stamp = timestamp32bits();

char daysOfTheWeekEN[7][4] = {
  "Sun",
  "Mon",
  "Tue",
  "Wed",
  "Thu",
  "Fri",
  "Sat"
};
char daysOfTheWeekIT[7][4] = {
  "Dom",
  "Lun",
  "Mar",
  "Mer",
  "Gio",
  "Ven",
  "Sab"
};

/* BLE MANAGER ---------------------------------------------------------------*/
//#include <BLEDevice.h>      // Controller Bluetooth
//#include <BLEServer.h>      // Server Bluetooth
//#include "esp_bt_device.h"
//#include <BLEUtils.h>       // Istruzioni aggiuntive Bluetooth

/* LIBRARY  ------------------------------------------------------------------*/
#include <TridentTD_Base64.h>
uint8_t templateBuffer[257280];

/*  DISPLAY --------------------------------------------------------------*/
//https://heltec-automation-docs.readthedocs.io/en/latest/esp32+arduino/quick_start.html#via-arduino-board-manager
//#define HelTecEnable
#ifdef HelTecEnable
#include "heltec.h"
SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED, RST_OLED);
//#include "images.h"
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16
#endif

#define SPREnable
#ifdef SPREnable

#include "rm67162.h"
#include <TFT_eSPI.h>  //https://github.com/Bodmer/TFT_eSPI
#include "true_color.h"
#include "image.h"
//#include <SoftwareSerial.h>
// #include <JPEGDecoder.h>

#if ARDUINO_USB_CDC_ON_BOOT != 1
#warning "If you need to monitor printed data, be sure to set USB CDC On boot to ENABLE, otherwise you will not see any data in the serial monitor"
#endif

#ifndef BOARD_HAS_PSRAM
#error "Detected that PSRAM is not turned on. Please set PSRAM to OPI PSRAM in ArduinoIDE"
#endif
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

#define WIDTH 536
#define HEIGHT 240
#define LINEHEIGHT 48
#endif
#include <JPEGDecoder.h>  // Include the JPEG decoder library


/* DIGITAL CALIPER AND BARCODE SCANNER PARAMETERS ----------------------------*/
int i;
int sign;
long value;
float measure;
bool triggerStatus = HIGH;
bool bootPinStatus = HIGH;
bool scanPinStatus = HIGH;
bool zeroPinStatus = HIGH;

String lastBarcode = "";

unsigned long tempmicros;

String id = "";
String descrizione = "";
String minimo = "";
String massimo = "";
String image_b64 = "";

/* PIN -----------------------------------------------------------------------*/
#define BOOTPIN 0  // boot button pin
#define LEDPIN 38  // green led pin

//#define CALIPERPIN    13  // data pin digital caliper
#define ZEROPIN 3    // pin for caliper zeroing
#define CLOCKPIN 12  // clock pin digital caliper

//#define SCANPIN    21  // scan button pin
#define TRIGGERPIN 14  // trigger pin barcode scanner
#define TX_PIN 43      // tx pin barcode scanner
#define RX_PIN 44      // rx pin barcode scanner


// PARAMETRI
#define WDT_TIMEOUT 550     // Timeout del watch-dog [s]
#define ConnectionInt 3600  // Intervallo di ricerca reti e connessione [s]
#define PAR_SerBaud 115200  // Velocità della comunicazione seriale. [300 - 2000000 baud]
#define PAR_T0Int 1000      // Specifica ogni quanto tempo chiamare la routine di interrupt sul Timer 0, dove vengono generati i clock di sistema. [microsecondi]
#define PAR_AIRes 10        // Risoluzione degli ingressi analogici. [8 - 12 bit. Se si modifica, correggere i parametri di conversione della lettura]
#define PAR_ADCAtt 0        // Attenuazione degli ingressi analogici. [0 = OFF, 1 = 2.5dB (1V = 2086), 2 = 6dB (1V = 2975), 3 = 11dB (1V = 3959)]

#define PUSHTIME 2000  // Tempo per cui il pulsante di misura deve essere premuto per avere la funzione di azzeramento

String jsonString;

unsigned int PREV_MILLIS_BTN = 0;
unsigned int PREV_MILLIS_CONN = 0;
unsigned int PREV_MILLIS_SECOND = 0;

/* WIFI LAN ------------------------------------------------------------------*/
#ifdef WifiEnable
bool WifiUsable = false;
#include <esp_wifi.h>
#include <WiFi.h>
#endif

/* ETHERNET LAN --------------------------------------------------------------*/
#ifdef EthEnable
bool EthUsable = false;
#include <SPI.h>
#include <Ethernet.h>
#endif

/* GPRS LAN ------------------------------------------------------------------*/
#ifdef GprsEnable
bool ModemUsable = false;
bool GPRSUsable = false;
//SoftwareSerial gprsSerial(10, 11); // RX, TX pins on Arduino
#define MODEM_TX 11
#define MODEM_RX 10
#define TINY_GSM_MODEM_SIM7600  // GSM module
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 1024
#endif
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200
#define TINY_GSM_DEBUG Serial
#define SerialGSM Serial2  // Modulo GSM
#include <TinyGsmClient.h>
TinyGsm modem(SerialGSM);
#endif

/* COMMON TCP ----------------------------------------------------------------*/
#include <ESPmDNS.h>
#include <DNSServer.h>

#ifdef WifiEnable
WiFiClient w_client;
#endif
#ifdef EthEnable
EthernetClient e_client;
#endif
#ifdef GprsEnable
TinyGsmClient g_client(modem, 0);
#endif
DNSServer dnsServer;
#include <ArduinoHttpClient.h>
String callURL(const String &Server, const String &url, const String &body);
String convertObjectToString(const void *data);


/* SERVER WEB ----------------------------------------------------------------*/
#include <aWOT.h>
#ifdef EthEnable
EthernetServer EthServer(80);
#endif
#ifdef WifiEnable
WiFiServer WifServer(80);
#endif
Application app;

/* HTTP WEB ------------------------------------------------------------------*/
#include <ArduinoHttpClient.h>

/* TIME SYNC RTC      --------------------------------------------------------*/
uint8_t bin2bcd(uint8_t val) {
  return val + 6 * (val / 10);
}
uint8_t bcd2bin(uint8_t val) {
  return val - 6 * (val >> 4);
}
void syncTimeFromUTC(String TZString = "Europe/Rome", time_t utcTimestamp = 0) {
  Serial.println("TIME Syncing time ...");
  if (TZString.isEmpty()) return;

  unsetenv("TZ");
  tzset();

  if (utcTimestamp > 1689263334) {
    Serial.println("TIME Syncing time RTC INT...");
    rtc.setTime(utcTimestamp);

    Wire.beginTransmission(0x68);
    //il primo byte stabilisce il registro iniziale da scivere
    Wire.write((byte)0x00);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 1);
    uint8_t ch;  //,d;
    ch = Wire.read();
    //d = Wire.read();
    if (ch & 0x80) {
      ch = 0x80;
    } else {
      ch = 0x00;
    }
    //specifico il tempo e la data
    Wire.beginTransmission(0x68);
    Wire.write(byte(0x00));
    Wire.write(bin2bcd(rtc.getSecond() | ch));    //1° byte SECONDI da 0x00 a 0x59
    Wire.write(bin2bcd(rtc.getMinute()));         //2° byte MINUTI da 0x00 a 0x59
    Wire.write(bin2bcd(rtc.getHour(true)));       //3° byte ORE da 0x00 a 0x23
    Wire.write(bin2bcd(0));                       //4° byte GIORNO della settimana da 0x01 a 0x07
    Wire.write(bin2bcd(rtc.getDay()));            //5° byte GIORNO del mese da 0x01 a 0x31
    Wire.write(bin2bcd(rtc.getMonth() + 1));      //6° byte MESE da 0x01 a 0x12
    Wire.write(bin2bcd((rtc.getYear() - 2000)));  //7° byte ANNO 0x00 a 0x99
    Wire.endTransmission();
    Serial.println("TIME Time updated in DS1307 in UTC!");
  }

  Serial.println("TIME Syncing time RTC EXT...");
  Wire.beginTransmission(0x68);
  Wire.write((byte)0x00);
  Wire.endTransmission();

  //richiedo 7 byte dal dispositivo con indirizzo 0x68
  Wire.requestFrom(0x68, 7);
  int sec = bcd2bin(Wire.read() & 0x7F);
  int min = bcd2bin(Wire.read());
  int hour = bcd2bin(Wire.read());
  int wday = bcd2bin(Wire.read());  //day of week
  int day = bcd2bin(Wire.read());
  int month = bcd2bin(Wire.read());
  int year = bcd2bin(Wire.read()) + 2000;
  rtc.setTime(sec, min, hour, day, month, year, 0);

  setenv("TZ", findTimeOffset(TZString), 1);
  tzset();
  Serial.println(rtc.getTime("RTC0: %A, %B %d %Y %H:%M:%S"));
  Serial.println("TIME Time synced from NTP!");
}

const char *findTimeOffset(const char *searchKey) {
  for (int i = 0; i < TimeZonesRows; i++) {
    if (strcmp(TimeZones[i][0], searchKey) == 0) {
      return TimeZones[i][1];
    }
  }
  return nullptr;
}
const char *findTimeOffset(const String &searchKey) {
  for (int i = 0; i < TimeZonesRows; i++) {
    if (searchKey.equals(TimeZones[i][0])) {
      return TimeZones[i][1];
    }
  }
  return nullptr;
}


/* SCHEDULER -----------------------------------------------------------------*/
// void WeeklySchedulerFunction(String FuncName, unsigned long duration, unsigned long repeatevery);
// #include "WeeklyScheduler.h"
// WeeklyScheduler scheduler;

#define SCHEnable
#ifdef SCHEnable
void WeeklySchedulerFunction(String FuncName, unsigned long duration, unsigned long repeatevery);
#include "WeeklyScheduler.h"
WeeklyScheduler scheduler;

/* SCHEDULER FUNCTION ----------------------------------------------------*/
void SchedulerRx() {
  Serial.println("Scheduled load tasks (JSON):");
  String jsonString = callURL(geqo_url, (String) "/index.php?" + "api=" + geqo_api + "&" + "mac=" + chipId, "");

  if (!jsonString.isEmpty()) {
    Serial.println("[SchedulerRxHTTP]:");
    Serial.println(jsonString);

    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
      Serial.print("Scheduler Failed to parse JSON: ");
      Serial.println(error.c_str());
      return;
    }

    //timing update align
    settings.begin("geqo", false);

    int uValue = doc["U"].as<int>();
    if (uValue > 3600) uValue = 3600;
    if (uValue < 10) uValue = 10;

    geqo_align = String(uValue);
    Serial.println(geqo_align);
    settings.putString("geqo_align", geqo_align);

    int dValue = doc["D"].as<int>();
    geqo_zone = doc["Z"].as<String>();
    Serial.println(geqo_zone);
    settings.putString("geqo_zone", geqo_zone);
    settings.end();

    syncTimeFromUTC(geqo_zone, dValue);

    iotName = doc["DD"].as<String>();
    Serial.println(iotName);
    settings.putString("iotname", iotName);

    //PumpDutyCicle = doc["PP"].as<int>();
    //settings.putInt("PumpDutyCicle", PumpDutyCicle);

    //AirSpeedTrig = doc["AS"];
    //settings.putFloat("AirSpeedTrig", AirSpeedTrig);

    JsonArray taskArray = doc["tasks"];
    scheduler.deleteAllTasks();
    for (const auto &taskObject : taskArray) {
      byte dayOfWeek = taskObject["D"];
      String start = taskObject["S"];
      String end = taskObject["E"];
      unsigned long repeatEvery = taskObject["R"];
      unsigned long duration = taskObject["T"];
      String callbackName = taskObject["N"];

      // Create a new task
      scheduler.addTask(dayOfWeek, start, end, repeatEvery, duration, callbackName);
    }
  } else {
    Serial.println("[SchedulerRxHTTP]: NO RESPONSE");
  }
}

void SchedulerTx() {
  String jsonString = scheduler.toJSON();
  if (!jsonString.isEmpty()) {
    String response = callURL(geqo_url, (String) "/index.php?" + "api=" + geqo_api + "&" + "mac=" + chipId + "&" + "machine=" + geqo_machine + "&" + "scheduler=" + jsonString, "");
    Serial.println("[SchedulerTxHTTP]:" + response);
  }
}

void WeeklySchedulerFunction(String FuncName, unsigned long duration, unsigned long repeatevery) {
  if (FuncName == "EV1") {
  } else if (FuncName == "EV2") {
  } else if (FuncName == "MON") {
  } else {
    Serial.println("Error: Unknown function");
  }
}
#endif

/* WEB SERVER FUNCTION -------------------------------------------------------*/
char redirectURL[30];

void redirect(Request &req, Response &res) {
  Serial.println("WEB redirect");
  //Serial.println(req);
  if (!res.statusSent() && (redirectURL != "")) {
    res.set("Location", redirectURL);
    res.sendStatus(302);
    //sprintf(redirectURL,  "");
  }
}
void popup(Request &req, Response &res) {
  Serial.println("WEB popup");
  handle_root(req, res);
}
void notFound(Request &req, Response &res) {
  Serial.println("WEB notFound");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<h1>Not Found!</h1>");
  res.println("</body>");
  res.println("</html>");
}

void handle_css(Request &req, Response &res) {
  Serial.println("WEB handle_css");
  res.set("Content-Type", "text/css");
  res.println("body { font-family: Arial, sans-serif; }");
  res.println("h1 { color: #333; text-align: center; }");
  res.println(".container { border: 1px solid #ccc; margin: auto;  margin-bottom: 20px; margin: 5px; }");
  res.println(".logo { width:50px; height:50px; margin: 20px;border:1px solid black;display: flex;align-items: center;justify-content: center; }");
  res.println(".btn-group  {  margin:auto; text-align: center; }");
  res.println(".button { background-color: #1c87c9; border: none; color: white; padding: 15px 5px; border-radius: 6px; text-align: center; text-decoration: none; display: inline-block; font-size: 20px; margin: 4px 2px; width: 100px; cursor: pointer;}");
  res.println(".status  { border: 1px solid #ccc; margin: auto;  margin-bottom: 20px; margin: 5px; }");
  res.println(".status b { font-weight: bold;}");
  res.println(".status p { margin: 5px 0; }");

  res.end();
}

void handle_manifest(Request &req, Response &res) {
  Serial.println("WEB handle_css");
  res.set("Content-Type", "text/json");
  res.println("{");
  res.println("\"name\": \"SmartCaliper\",");
  res.println("\"short_name\": \"SmartCaliper\",");
  res.println("\"start_url\": \"index.html\",");
  res.println("\"display\": \"standalone\",");
  res.println("\"background_color\": \"#fdfdfd\",");
  res.println("\"theme_color\": \"#db4938\",");
  res.println("\"orientation\": \"portrait-primary\",");
  res.println("\"icons\": [");
  res.println("{\"src\": \"/images/icons/icon-96x96.png\",\"type\": \"image/png\", \"sizes\": \"96x96\"},");
  res.println("{\"src\": \"/images/icons/icon-128x128.png\",\"type\": \"image/png\", \"sizes\": \"128x128\"},");
  res.println("]");
  res.println("}");
  res.end();
}

void handle_root(Request &req, Response &res) {
  Serial.println("WEB handle_root");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<link rel='manifest' href='manifest.json' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<div id='block_container'>");
  res.println("<div class='logo'>");
  res.println("<h1>IF</h1>");
  res.println("</div>");
  res.println("<div >");
  res.println("<h1>Welcome to SmartCaliper</h1>");
  res.println("</div>");
  res.println("<div >");
  res.println("<h1>" + iotName + "</h1>");
  res.println("</div>");
  res.println("<div >");
  res.println("<h2>" + chipId + "</h1>");
  res.println("</div>");
  res.println("</div>");

  res.println("<div class='status'>");
  res.println("<b>Current Ver:</b>");
  res.println(FWversion);
  res.println("<BR>");

  res.print("<p>Date:");
  res.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
  res.println("</p>");

  res.print("<p>GMT Zone:");
  res.println(geqo_zone);
  res.println("</p>");

  res.print("<p>GMT Rule:");
  res.println(findTimeOffset(geqo_zone));
  res.println("</p>");

  res.print("<p>Align Every:");
  res.println(geqo_align);
  res.println("</p>");

  res.println("<p>MAC:" + chipId + "</p>");

#ifdef EthEnable
  res.println("<p>Ethernet:");
  if (EthUsable) {
    res.println((String) "OK IP:" + Ethernet.localIP().toString().c_str() + "</p>");
  } else {
    res.println("KO</p>");
  }
#endif

#ifdef WifiEnable
  res.println("<p>Wifi:");
  if (WifiUsable) {
    res.println((String) "OK IP:" + WiFi.localIP().toString().c_str() + " RSSI:" + WiFi.RSSI() + "</p>");
  } else {
    res.println("KO</p>");
  }
#endif

#ifdef GprsEnable
  res.println("<p>GPRS:");
  if (GPRSUsable) {
    res.println((String) "OK IP:" + modem.localIP().toString().c_str() + " RSSI:" + modem.getSignalQuality() + "</p>");
  } else {
    res.println("KO</p>");
  }
  res.println("</div>");
#endif


  res.println("<div class='container' >");

  res.println("<div class='btn-group'>");
  res.println("<a href='/ethconfig' class='button'>Ethernet</a>");
  res.println("<a href='/wificonfig' class='button'>WiFi</a>");
  res.println("<a href='/gprsconfig' class='button'>GPRS</a>");
  res.println("</div>");

  res.println("<div class='btn-group'>");
  res.println("<a href='/srvconfig' class='button'>SetUP</a>");
  res.println("<a href='/schedulerconfig' class='button'>Scheduler</a>");
  res.println("<a href='/sensors' class='button'>Sensors</a>");
  res.println("</div>");

  res.println("<div class='btn-group'>");
  res.println("<a href='/upload' class='button'>Firmware</a>");
  res.println("<a href='/reset' class='button'>Reset</a>");
  res.println("<a href='/reconnect' class='button'>Reconnect</a>");
  res.println("</div>");

  res.println("</div>");


  res.println("</body></html>");
  res.end();
}
void handle_reset(Request &req, Response &res) {
  Serial.println("WEB handle_reset");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  // Restart the device
  ESP.restart();

  res.println("</body></html>");
  res.end();
}

void handle_SrvConfig(Request &req, Response &res) {
  Serial.println("WEB handle_setup");
  settings.begin("geqo", false);

  char value[64];
  int32_t valueint;
  char name[64];

  setStringToArray(name, "geqo_url");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_url");
    Serial.println(value);
    settings.putString("geqo_url", value);
    geqo_url = String(settings.getString("geqo_url", "").c_str());
  }

  setStringToArray(name, "geqo_machine");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_machine");
    Serial.println(value);
    settings.putString("geqo_machine", value);
    geqo_machine = String(settings.getString("geqo_machine", "").c_str());
  }

  setStringToArray(name, "geqo_user");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_user");
    Serial.println(value);
    settings.putString("geqo_user", value);
    geqo_user = String(settings.getString("geqo_user", "").c_str());
  }

  setStringToArray(name, "geqo_pwd");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_pwd");
    Serial.println(value);
    settings.putString("geqo_pwd", value);
    geqo_pwd = String(settings.getString("geqo_pwd", "").c_str());
  }

  setStringToArray(name, "geqo_dbname");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_dbname");
    Serial.println(value);
    settings.putString("geqo_dbname", value);
    geqo_dbname = String(settings.getString("geqo_dbname", "").c_str());
  }

  setStringToArray(name, "geqo_align");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_align");
    Serial.println(value);
    settings.putString("geqo_align", value);
    geqo_align = String(settings.getString("geqo_align", "").c_str());
  }

  setStringToArray(name, "geqo_api");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_api");
    Serial.println(value);
    settings.putString("geqo_api", value);
    geqo_api = String(settings.getString("geqo_api", "").c_str());
  }

  setStringToArray(name, "geqo_zone");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_zone");
    Serial.println(value);
    settings.putString("geqo_zone", value);
    geqo_zone = String(settings.getString("geqo_zone", "").c_str());
  }

  settings.end();


  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  settings.begin("geqo", false);

  res.println("<form action='/srvconfig' method='POST'>");
  res.println("<table id='setup' border='1' style='border: 1px solid black;' >");
  res.println("<tr>");
  res.println("<th>Var</th>");
  res.println("<th>Value</th>");
  res.println("</tr>");

  res.println("<tr>");
  res.println("<td>geqo_url</td>");
  res.println("<td><input name='geqo_url' length=32 value='" + geqo_url + "'></td>");
  res.println("</tr>");

  res.println("<tr>");
  res.println("<td>geqo_machine</td>");
  res.println("<td><input name='geqo_machine' length=32 value='" + geqo_machine + "'></td>");
  res.println("</tr>");

  res.println("<tr>");
  res.println("<td>geqo_user</td>");
  res.println("<td><input name='geqo_user'  length=32  value='" + geqo_user + "'></td>");
  res.println("</tr>");

  res.println("<tr>");
  res.println("<td>geqo_pwd</td>");
  res.println("<td><input name='geqo_pwd' length=32 value='" + geqo_pwd + "'></td>");
  res.println("</tr>");

  res.println("<tr>");
  res.println("<td>geqo_dbname</td>");
  res.println("<td><input name='geqo_dbname' length=32 value='" + geqo_dbname + "'></td>\n");
  res.println("</tr>");

  res.println("<tr>");
  res.println("<td>geqo_align</td>");
  res.print("<td><input name='geqo_align' length=32 value='");
  res.print(geqo_align);
  res.println("'></td>");
  res.println("</tr>");

  res.println("<tr>");
  res.println("<td>geqo_api</td>");
  res.println("<td><input name='geqo_api' length=32 value='" + geqo_api + "'></td>");
  res.println("</tr>");

  res.println("<tr>");
  res.println("<td>geqo_zone</td>");

  res.println("<td><select id='geqo_zone' name='geqo_zone'>");
  for (int i = 0; i < TimeZonesRows; i++) {
    res.print("<option value='");
    res.print(TimeZones[i][0]);
    res.print("'");
    if (geqo_zone.equals(TimeZones[i][0])) res.print(" selected");
    res.print(">");
    res.print(TimeZones[i][0]);
    res.println("</option>");
  }
  res.println("</select></td>");
  res.println("</tr>");

  res.println("</table>");
  res.println("<BR>");
  res.println("<input class='button' type='submit'>");
  res.println("</form>");

  res.println("</html>");
  res.end();
  settings.end();
}

#ifdef WifiEnable
void handle_WiFiConfig(Request &req, Response &res) {
  Serial.println("WEB handle_WiFiConfig");
  settings.begin("wifi_config", false);

  char value[64];
  char name[64];

  setStringToArray(name, "ssid");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("ssid", value);
  }

  setStringToArray(name, "password");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("password", value);

    connectToInet();
  }

  res.set("Content-Type", "text/html");
  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<h1>Wi-Fi Configuration</h1>");

  res.println("<h2>Available Networks:</h2>");
  res.println("<ul>");
  int numOfNetworks = WiFi.scanNetworks();
  for (int i = 0; i < numOfNetworks; i++) {
    res.print("<li>");
    res.print(WiFi.SSID(i));
    res.print(" ->(dB)");
    res.print(WiFi.RSSI(i));
    res.println("</li>");
  }
  res.println("</ul>");

  res.println("<h2>Configure Wi-Fi:</h2>");
  res.println("<form action='/wificonfig' method='POST'>");
  res.println("SSID:<br><input type='text' name='ssid' value='" + String(settings.getString("ssid", "").c_str()) + "'><br>");
  res.println("Password:<br><input type='password' name='password' value='" + String(settings.getString("password", "").c_str()) + "'><br>");
  res.println("<input type='submit' class='button' value='Submit'>");
  res.println("</form>");

  settings.end();
  res.end();
}
#endif

#ifdef EthEnable
void handle_EthConfig(Request &req, Response &res) {
  Serial.println("WEB handle_EthConfig");
  settings.begin("eth_config", false);

  char value[64];
  char name[64];

  setStringToArray(name, "IP");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("IP", value);
  }

  setStringToArray(name, "GW");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("GW", value);
  }

  setStringToArray(name, "SM");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("SM", value);
  }

  setStringToArray(name, "DNS");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("DNS", value);

    connectToInet();
  }

  res.set("Content-Type", "text/html");
  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<h1>Eth Configuration</h1>");

  res.println("<ul>");
  res.println("</ul>");

  res.println("<h2>Configure Eth:</h2>");
  res.println("<form action='/ethconfigset' method='POST'>");
  res.println("IP:<br><input type='text'  name='IP'  value='" + String(settings.getString("IP", "")) + "'><br>");
  res.println("SM:<br><input type='text'  name='SM'  value='" + String(settings.getString("SM", "")) + "'><br><br>");
  res.println("GW:<br><input type='text'  name='GW'  value='" + String(settings.getString("GW", "")) + "'><br><br>");
  res.println("DNS:<br><input type='text' name='DNS' value='" + String(settings.getString("DNS", "")) + "'><br><br>");
  res.println("<input type='submit' class='button' value='Submit'>");
  res.println("</form>");

  settings.end();
  res.end();
}
#endif

#ifdef GprsEnable
void handle_GprsConfig(Request &req, Response &res) {
  Serial.println("WEB handle_GprsConfig");
  char value[64];
  char name[64];

  setStringToArray(name, "PIN");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    GPRS_PIN = value;
    settings.putString("GPRS_PIN", value);
  }

  setStringToArray(name, "APN");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    GPRS_APN = value;
    settings.putString("GPRS_APN", value);
  }

  setStringToArray(name, "LOGIN");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    GPRS_LOGIN = value;
    settings.putString("GPRS_LOGIN", value);
  }

  setStringToArray(name, "PASSWORD");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    GPRS_PASSWORD = value;
    settings.putString("GPRS_PASSWORD", value);
    connectToInet();
  }

  res.set("Content-Type", "text/html");
  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  settings.begin("gprs_config", false);

  res.println("<h1>GPRS Configuration</h1>");

  res.println("<ul>");

  if (modem.isNetworkConnected()) {
    res.println("Network connected");
    res.println("<br>");
  }

  if (modem.isGprsConnected()) {
    res.println("GPRS status: connected");
    res.println("<br>");
  }

  String ccid = modem.getSimCCID();
  res.println("CCID:" + ccid);
  res.println("<br>");

  String imei = modem.getIMEI();
  res.println("IMEI:" + imei);
  res.println("<br>");

  String imsi = modem.getIMSI();
  res.println("IMSI:" + imsi);
  res.println("<br>");

  String cop = modem.getOperator();
  res.println("Operator:" + cop);
  res.println("<br>");

  IPAddress modemIP = modem.localIP();
  res.println((String) "Local IP:" + modemIP.toString().c_str());
  res.println("<br>");

  int csq = modem.getSignalQuality();
  res.println("Signal quality:" + csq);
  res.println("<br>");

  res.println("</ul>");

  res.println("<h2>Configure GPRS:</h2>");
  res.println("<form action='/gprsconfigset' method='POST'>");
  res.println("PIN:<br><input type='text'      name='PIN'      value='" + String(GPRS_PIN) + "'><br>");
  res.println("APN:<br><input type='text'      name='APN'      value='" + String(GPRS_APN) + "'><br><br>");
  res.println("LOGIN:<br><input type='text'    name='LOGIN'    value='" + String(GPRS_LOGIN) + "'><br><br>");
  res.println("PASSWORD:<br><input type='text' name='PASSWORD' value='" + String(GPRS_PASSWORD) + "'><br><br>");
  res.println("<input type='submit' class='button' value='Submit'>");
  res.println("</form>");

  settings.end();
  res.end();
}
#endif

void handle_reconnect(Request &req, Response &res) {
  Serial.println("WEB handle_align");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  /* ACTIVATE */
  res.println("<br><h1>Connessione Dati</h1><br>");
  res.println((String) "connectToInet" + "<br>");
  connectToInet();

  res.println("<br><h1>Trasmissione Dati</h1><br>");
  String URLString = (String) "/index.php?" + "api=" + geqo_api + "&" + "mac=" + chipId + "&" + "hb=1";
  res.println("<pre>" + URLString + "</pre>");

  esp_task_wdt_reset();  // Reset del watch-dog
  String response = callURL(geqo_url, URLString, "");

  res.println("<br><h1>Ricezione Dati</h1><br>");
  res.println((String) "<pre>" + response + "</pre>");

  res.println("<br><h1>Syncing date time</h1><br>");
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("TIME SYNC:");
  if (!response.isEmpty()) {
    Serial.println("TIME HTTP:" + response);
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, response);
    if (error) {
      Serial.println((String) "TIME Scheduler Failed to parse JSON: " + error.c_str());
      res.println((String) "<h1>TIME Scheduler Failed to parse JSON: " + error.c_str() + "</h1>");
    } else {
      //timing update align
      Serial.println("convert json to var");
      settings.begin("geqo", false);

      int uValue = doc["U"].as<int>();
      if (uValue > 3600) uValue = 3600;
      if (uValue < 10) uValue = 10;

      geqo_align = String(uValue);
      Serial.println(geqo_align);
      settings.putString("geqo_align", geqo_align);

      int dValue = doc["D"].as<int>();
      geqo_zone = doc["Z"].as<String>();
      Serial.println(geqo_zone);
      settings.putString("geqo_zone", geqo_zone);
      settings.end();

      syncTimeFromUTC(geqo_zone, dValue);

      iotName = doc["DD"].as<String>();
      Serial.println(iotName);
      settings.putString("iotname", iotName);

      //date time align
      syncTimeFromUTC(geqo_zone, dValue);
      res.println("<h1>Time Synced</h1>");
    }
  } else {
    Serial.println("TIME HTTP: NO RESPONSE");
    syncTimeFromUTC(geqo_zone, 0);
  }

  res.end();
}

/* UPDATE ---------------------------------------------------------------*/
#include <Update.h>
void handle_upload(Request &req, Response &res) {
  Serial.println("WEB handle_upload");

  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("  <h1>");
  res.println("    Compiled: " __DATE__ " " __TIME__);
  res.println("  </h1>");
  res.println("  <form id='form'>");
  res.println("    <input id='file' type='file'>");
  res.println("    <input type='submit' value='Send' />");
  res.println("  </form>");
  res.println("</body>");
  res.println("<script>");
  res.println("  const form = document.getElementById('form');");
  res.println("  form.onsubmit = function(e) {");
  res.println("    e.preventDefault();");
  res.println("    const body = document.getElementById('file').files[0];");
  res.println("    fetch('/update', { method: 'POST', body }).then((response) => {");
  res.println("      if (!response.ok) {");
  res.println("        return alert('File upload failed');");
  res.println("      }");
  res.println("      alert('File upload succeeded');");
  res.println("    });");
  res.println("  }");
  res.println("</script>");
  res.println("</html>");
}
void handle_update(Request &req, Response &res) {
  Serial.println("WEB handle_update");
  int contentLength = req.left();
  Serial.println("Dim:" + contentLength);

  if (strcmp(req.get("Expect"), "100-continue") == 0) {
    res.status(100);
  }

  Serial.println("OTA Update...");
  if (!Update.begin(contentLength)) {
    res.status(500);
    return Update.printError(req);
  }

  Serial.println("OTA Reboot...");
  unsigned long start = millis();
  while (!req.available() && millis() - start <= 5000) {}

  if (!req.available()) {
    return res.sendStatus(408);
  }

  Serial.println("OTA write...");
  if (Update.writeStream(req) != contentLength) {
    res.status(400);
    return Update.printError(req);
  }

  Serial.println("OTA end...");
  if (!Update.end(true)) {
    res.status(500);
    return Update.printError(req);
  }

  res.sendStatus(204);
}

/* SENSOR ---------------------------------------------------------------*/
void handle_Sensors(Request &req, Response &res) {
  Serial.println("WEB handle_align");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  /*
  res.println("<div>");
  
    res.println("<div style='width:49%; border:1px solid; float:left; clear:none;'>");
    
    res.println("<b>Current</b><BR>");
    res.println("<table id='udc' border='1' style='width:100%; border: 1px solid black;' >");
    res.println("<tr>");
    res.println("<th>Var</th>" + "<th>Value</th>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>DataOra</td>" + "<td>" + curTag.DataOra + "</td>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>PosLatitude</td>" + "<td>" + curTag.PosLatitude + "</td>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>PosLongitude</td>" + "<td>" + curTag.PosLongitude + "</td>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>PosCode</td>" + "<td>" + curTag.PosCode + "</td>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>PalCode</td>" + "<td>" + curTag.PalCode + "</td>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>PalStatus</td>" + "<td>" + curTag.PalStatus + "</td>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>PlannerID</td>" + "<td>" + curTag.PlannerID + "</td>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>Operator</td>" + "<td>" + geqo_Operator + "</td>");
    res.println("</tr>");
    res.println("</table>");
    res.println("<BR>");

    res.println("<b>Counting</b><BR>");
    res.println("<table id='position' border='1' style='width:100%; border: 1px solid black;' >");
    res.println("<tr>");
    res.println("<th>Var</th>" + "<th>Value</th>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>Pulse</td>" + "<td>" + pulseCount + "</td>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>Freq</td>" + "<td>" + pulseFreq + "</td>");
    res.println("</tr>");
    res.println("<tr>");
    res.println("<td>FreqAvg</td>" + "<td>" + pulseFreqAvg + "</td>");
    res.println("</tr>");
    res.println("</table><BR>");
    
    res.println("</div>");
  
    //STACK TAG
    res.println("<div style='width:49%; border:1px solid; float:left; clear:none;'>");
    res.println("<b>TagInMemory</b><BR>");
    EpromTags  stackTag;
    if(eeAddress>255){
      for (int i=255; i<eeAddress; i=i+sizeof(EpromTags)){
        EEPROM.get( i, stackTag );
        res.println("<b>Tag" + i + "</b><BR>");
        res.println("<table id='stack_ " + i + "' border='1' style='width:100%; border: 1px solid black;' >");
        res.println("<tr>");
        res.println("<th>Var</th>" + "<th>Value</th>");
        res.println("</tr>");
        res.println("<tr>");
        res.println("<td>DataOra</td>" + "<td>" + stackTag.DataOra + "</td>");
        res.println("</tr>");
        res.println("<tr>");
        res.println("<td>PosLatitude</td>" + "<td>" + stackTag.PosLatitude + "</td>");
        res.println("</tr>");
        res.println("<tr>");
        res.println("<td>PosLongitude</td>" + "<td>" + stackTag.PosLongitude + "</td>");
        res.println("</tr>");
        res.println("<tr>");
        res.println("<td>PosCode</td>" + "<td>" + stackTag.PosCode + "</td>");
        res.println("</tr>");
        res.println("<tr>");
        res.println("<td>PalCode</td>" + "<td>" + stackTag.PalCode + "</td>");
        res.println("</tr>");
        res.println("<tr>");
        res.println("<td>PalStatus</td>" + "<td>" + stackTag.PalStatus + "</td>");
        res.println("</tr>");
        res.println("<tr>");
        res.println("<td>PlannerID</td>" + "<td>" + stackTag.PlannerID + "</td>");
        res.println("</tr>");
        res.println("</table><BR>");
      }
    }
    res.println("</div>");
    
  res.println("<div>");
  */
  res.println("</body></html>");
  res.end();
}
void handle_SensorsRx(Request &req, Response &res) {
  Serial.println("WEB handle_SensorsRx");
  //call parent
  handle_Sensors(req, res);
}

/* SCHEDULER -------------------------------------------------------------*/
#ifdef SCHEnable
void handle_SchedulerConfig(Request &req, Response &res) {
  Serial.println("WEB handle_SchedulerConfig");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SmartCaliper</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<h1>SCHEDULER</h1>");
  res.println("<div style='width:100%; border:1px solid; float:left; clear:none;'>");

  // Display current time from RTC
  res.println("<b>Current</b><BR>");
  res.print("WeekDay:");
  //DAFARE rtc.
  res.println(daysOfTheWeekEN[1]);
  res.print("Date:");
  res.println(rtc.getTime("RTC0: %A, %B %d %Y %H:%M:%S"));
  res.print("<BR>");
  res.println("<a href='schedulerrx'>Get Scheduler From Server</a>");

  res.println("<form action='schedulerconfigset' method='post'>");
  res.println("<table id='udc' border='1' style='width:100%; border: 1px solid black;' >");
  res.println("<tr>");
  res.println("<td>WDay</td>");
  res.println("<td>Start</td>");
  res.println("<td>End</td>");
  res.println("<td>Repeat</td>");
  res.println("<td>Durate</td>");
  res.println("<td>Out</td>");
  res.println("</tr>");


  for (byte i = 0; i < MAX_TASKS; i++) {
    res.println("<tr>");

    byte dayOfWeek = scheduler.tasks[i].dayOfWeek;
    byte startHour = scheduler.tasks[i].startHour;
    byte startMinute = scheduler.tasks[i].startMinute;
    byte endHour = scheduler.tasks[i].endHour;
    byte endMinute = scheduler.tasks[i].endMinute;
    unsigned long repeatEvery = scheduler.tasks[i].repeatEvery;
    unsigned long duration = scheduler.tasks[i].duration;
    String callbackName = scheduler.tasks[i].callbackName;

    /**  GIORNO **/
    res.println("<td>");
    res.print((String) "<select id='DW" + i + "' name='DW" + i + "'>");

    for (int d = 0; d < 7; d++) {
      res.print((String) "<option value='" + d + "'");
      if (dayOfWeek == d) res.print(" selected");
      res.println((String) ">" + daysOfTheWeekEN[d] + "</option>");
    }
    res.println("</select>");
    res.println("</td>");

    /**  INIZIO **/
    res.println((String) "<td><input type='time' id='SH" + i + "' name='SH" + i + "' value='" + padTime(startHour, startMinute) + "'></td>");

    /**  FINE **/
    res.print((String) "<td><input type='time' id='EH" + i + "' name='EH" + i + "' value='" + padTime(endHour, endMinute) + "'></td>");

    /**  RIPETIZIONE **/
    res.print((String) "<td><input type='number' id='RE" + i + "' name='RE" + i + "' min=0 max=65535 value='" + repeatEvery + "'></td>");

    /**  DURATA **/
    res.print((String) "<td><input type='number' id='DD" + i + "' name='DD" + i + "' min=0 max=65535 value='" + duration + "'></td>");

    /**  FUNZIONE **/
    res.println((String) "<td><select id='CB" + i + "' name='CB" + i + "'>");
    if (callbackName == "EV1") {
      res.println("<option value='EV1' selected>EV1</option>");
    } else {
      res.println("<option value='EV1' >EV1</option>");
    }
    if (callbackName == "EV2") {
      res.println("<option value='EV2' selected>EV2</option>");
    } else {
      res.println("<option value='EV2' >EV2</option>");
    }
    if (callbackName == "MON") {
      res.println("<option value='MON' selected>MON</option>");
    } else {
      res.println("<option value='MON' >MON</option>");
    }
    if (callbackName == "") {
      res.println("<option value='' selected>None</option>");
    } else {
      res.println("<option value='' >None</option>");
    }
    res.println("</select>");
    res.println("</td>");

    res.println("</tr>");
  }

  res.println("</tr>");
  res.println("</table>");

  res.println("<input type='submit'  class='button' value='Submit'>");
  res.println("</form>");


  res.println("</div>");
  res.println("</body></html>");

  res.end();
}

void handle_SchedulerConfigSet(Request &req, Response &res) {
  Serial.println("WEB handle_SchedulerConfigSet");
  char var[64];
  scheduler.deleteAllTasks();
  char dayOfWeek[64];
  char startHourMin[64];
  char endHourMin[64];
  char repeatEvery[64];
  char duration[64];
  char callbackName[64];
  char myString[3] = { '0' };
  char numberString[2] = { '0' };
  for (int i = 0; i < MAX_TASKS; i++) {

    itoa(i, numberString, 10);

    myString[0] = { 'D' };
    myString[1] = { 'W' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, dayOfWeek, 64);
    Serial.println(dayOfWeek);

    myString[0] = { 'S' };
    myString[1] = { 'H' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, startHourMin, 64);
    Serial.println(startHourMin);

    myString[0] = { 'E' };
    myString[1] = { 'H' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, endHourMin, 64);
    Serial.println(endHourMin);

    myString[0] = { 'R' };
    myString[1] = { 'E' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, repeatEvery, 64);
    Serial.println(repeatEvery);

    myString[0] = { 'D' };
    myString[1] = { 'D' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, duration, 64);
    Serial.println(duration);

    myString[0] = { 'C' };
    myString[1] = { 'B' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, callbackName, 64);
    Serial.println(callbackName);

    scheduler.addTask(atoi(dayOfWeek), startHourMin, endHourMin, atoi(repeatEvery), atoi(duration), callbackName);
    Serial.println('INSERTED');
  }
  //call parent
  handle_SchedulerConfig(req, res);
}
void handle_SchedulerRx(Request &req, Response &res) {
  Serial.println("WEB handle_SchedulerRx");
  SchedulerRx();
  //call parent
  handle_SchedulerConfig(req, res);
}
#endif

/* FUNCTION ------------------------------------------------------------------*/
String ipToString(IPAddress ip) {
  String s = "";
  for (int i = 0; i < 4; i++)
    s += i ? "." + String(ip[i]) : String(ip[i]);
  return s;
}
String mac2String(byte ar[]) {
  String s;
  for (byte i = 0; i < 6; ++i) {
    char buf[3];
    sprintf(buf, "%02X", ar[i]);  // J-M-L: slight modification, added the 0 in the format for padding
    s += buf;
    if (i < 5) s += ':';
  }
  return s;
}
void setStringToArray(char *arr, const char *str) {
  size_t size = sizeof(str);
  strncpy(arr, str, size - 1);
  arr[size - 1] = '\0';  // Add the null terminator manually
}
void setStringToArray(char outputArray[], const String &inputString) {
  int arraySize = inputString.length();
  for (int i = 0; i < inputString.length(); i++) {
    outputArray[i] = inputString.charAt(i);
  }
  outputArray[inputString.length()] = '\0';
}
void setStringToArray(char outputArray[], const String &inputString, int arraySize) {
  if (arraySize < inputString.length() + 1) {
    return;
  }
  for (int i = 0; i < inputString.length(); i++) {
    outputArray[i] = inputString.charAt(i);
  }
  outputArray[inputString.length()] = '\0';
}
String convertObjectToString(const void *data) {
  size_t dataSize = sizeof(data);
  const byte *rawData = reinterpret_cast<const byte *>(data);
  String result;
  for (size_t i = 0; i < dataSize; ++i) {
    if (i > 0) {
      result += "&";
    }
    result += String(rawData[i]);
  }
  return result;
}

#define minimum(a, b) (((a) < (b)) ? (a) : (b))
/* CALIPER FUNCTION ----------------------------------------------------------*/
void elapsedTime() {
  unsigned long currentTime = millis();         // inizializzo currentTime che serve per la funzione elapsedTime
  unsigned long minutes = currentTime / 60000;  // 1 minuto = 60,000 millisecondi
  unsigned long hours = minutes / (60000);
  unsigned long seconds = (currentTime / 1000) % 60;

  spr.fillRect(0, 0, WIDTH, LINEHEIGHT, TFT_BLACK);
  spr.setCursor(0, 0);
  spr.setTextSize(5);

  if (hours) {
    spr.print(hours);
    spr.print("h ");
    spr.print(minutes);
    spr.print("min ");
    spr.print(seconds);
    spr.print("s");
  }
  if (minutes) {
    spr.print(minutes);
    spr.print("min ");
    spr.print(seconds);
    spr.print("s");
  } else {
    spr.print(seconds);
    spr.print("s");
  }
  lcd_PushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
}

#ifdef SCANPIN
void scanCode() {

  lastBarcode = "";
  spr.fillRect(0, LINEHEIGHT, WIDTH - HEIGHT, LINEHEIGHT * 3, TFT_BLACK);
  spr.setCursor(0, LINEHEIGHT);
  spr.setTextSize(3);
  // spr.println("Item:");

  digitalWrite(TRIGGERPIN, !triggerStatus);
  delay(100);
  digitalWrite(TRIGGERPIN, triggerStatus);
  // Serial.println("cccc");
  // Serial.println("bbbb")
  while (lastBarcode == "") {
    digitalWrite(TRIGGERPIN, !triggerStatus);
    lastBarcode = Serial1.readString();
    spr.print(lastBarcode);
    // Serial.println("while loop");
    if (digitalRead(SCANPIN) == 1) {
      break;
    }
  }

  digitalWrite(TRIGGERPIN, triggerStatus);

  if (lastBarcode != "") {
    digitalWrite(LEDPIN, HIGH);
    delay(400);
    digitalWrite(LEDPIN, LOW);
    BarcodeTx();
  }

  lcd_PushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
}
#endif

#ifdef CALIPERPIN
void readCaliper() {
  sign = 1;
  value = 0;
  for (i = 0; i < 23; i++) {
    while (digitalRead(CLOCKPIN) == HIGH) {}  //wait until clock returns to HIGH- the first bit is not needed
    while (digitalRead(CLOCKPIN) == LOW) {}   //wait until clock returns to LOW
    if (digitalRead(CALIPERPIN) == LOW) {
      if (i < 20) {
        value |= 1 << i;
      }
      if (i == 20) {
        sign = -1;
      }
    }
  }
  measure = (value * sign) / 100.0;

  spr.fillRect(0, LINEHEIGHT * 4, WIDTH - HEIGHT, LINEHEIGHT, TFT_BLACK);
  spr.setCursor(0, LINEHEIGHT * 4);
  spr.setTextSize(5);
  spr.print(measure);
  spr.print(" mm");
  lcd_PushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
}

void zeroMeasure() {
  digitalWrite(ZEROPIN, LOW);
  delay(100);
  digitalWrite(ZEROPIN, HIGH);
}
#endif

/* WEB CLIENT FUNCTION -------------------------------------------------------*/
void connectToInet() {

/* CONNECTION WIFI */
#ifdef WifiEnable
  Serial.println("WIFI");
  esp_task_wdt_reset();  // Reset del watch-dog
  settings.begin("wifi_config", false);

  //DAFARE
  settings.putString("ssid", "Vodafone-123");
  settings.putString("password", "qwertyqwerty");

  if (settings.getString("ssid", "").isEmpty()) {
    WifiUsable = false;
    Serial.print("WiFi APMode: ");

    //captive portal inganno dns mettendo esp32 come server dns
    IPAddress apIP(192, 168, 123, 1);
    IPAddress netMsk(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, apIP, netMsk);
    WiFi.mode(WIFI_AP);
    WiFi.softAP((String) "" + WifiDefaultSSID + "_" + chipId, WifiDefaultPassword);
    Serial.print("Access Point IP address: ");
    Serial.println(WiFi.softAPIP());

    //dns relay to page
    IPAddress ip = WiFi.softAPIP();
    sprintf(redirectURL, "http://%d.%d.%d.%d/popup", ip[0], ip[1], ip[2], ip[3]);
    app.get("/popup", &popup);
    //app.use(&redirect);     //<-- alla accensione qlc url viene inoltrato qui, ma poi va disattivato .. c'è da scoprire come
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(53, "*", ip);
  } else {
    Serial.print("WiFi connecting: ");
    WiFi.begin(settings.getString("ssid", "").c_str(), settings.getString("password", "").c_str());
    unsigned long timeout = millis() + WifiTimeout;
    while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      WifiUsable = true;
      Serial.println("Connected");
    } else {
      Serial.println("NOT Connected");
    }
  }
  Serial.print("My WiFi address: ");
  Serial.println(WiFi.localIP());
#endif

/* CONNECTION ETHERNET */
#ifdef EthEnable
  Serial.println("ETHERNET");
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("Ethernet init");
  Ethernet.init(5);  // MKR ETH Shield
  switch (Ethernet.linkStatus()) {
    case Unknown:
      EthUsable = false;
      Serial.println("Ethernet Unknown");
      break;
    case LinkON:
      Serial.println("Ethernet ON");
      settings.begin("eth_config", false);
      if (settings.getString("IP", "").isEmpty()) {
        Ethernet.begin(mac);
      } else {
        //byte mac[] =    { 0x90, 0xA2, 0xDA, 0x10, 0x38, 0x53 };
        EthIP.fromString(settings.getString("IP", ""));
        EthGW.fromString(settings.getString("GW", ""));
        EthDNS.fromString(settings.getString("DNS", ""));
        EthSM.fromString(settings.getString("SM", ""));
        Ethernet.begin(mac, EthIP, EthGW, EthDNS, EthSM);
      }
      if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        EthUsable = false;
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      } else if (Ethernet.linkStatus() == LinkOFF) {
        EthUsable = false;
        Serial.println("Ethernet cable is not connected.");
      } else {
        EthUsable = true;
        Serial.print("My Eth IP: ");
        Serial.println(Ethernet.localIP());
        Serial.print("My Eth MAC: ");
        for (int i = 0; i < 6; i++) {
          Serial.print(mac[i], HEX);
          if (i < 5) Serial.print(':');
        }
        Serial.println();
      }

      break;
    case LinkOFF:
      EthUsable = false;
      Serial.println("Ethernet OFF");
      break;
  }
#endif

/* CONNECTION GPRS */
#ifdef GprsEnable
  Serial.println("GPRS");
  esp_task_wdt_reset();  // Reset del watch-dog
  //per evitare che perda un mare di tempo a syncronizzare il modem se è gia a posto salta il test
  char APN[20];
  GPRS_APN.toCharArray(APN, GPRS_APN.length() + 1);
  char PASSWORD[20];
  GPRS_PASSWORD.toCharArray(PASSWORD, GPRS_PASSWORD.length() + 1);
  char LOGIN[20];
  GPRS_LOGIN.toCharArray(LOGIN, GPRS_LOGIN.length() + 1);
  char PIN[5];
  GPRS_PIN.toCharArray(PIN, GPRS_PIN.length() + 1);

  Serial.println("GPRS Connection APN:" + (String)APN + " APN:" + (String)LOGIN + " GPRS_PASSWORD:" + (String)PASSWORD + " GPRS_PIN:" + (String)PIN);
  if (!GPRSUsable) {

    if (!ModemUsable) {
      Serial.println("serial begin");
      Serial.println("wait 10sec hope modem started...");
      TinyGsmAutoBaud(SerialGSM, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);

      Serial.println("modem init");
      if (!modem.init()) {
        //Serial.println("modem restart");
        //if (!modem.restart()) {
        Serial.println("Failed to restart modem, delaying 10s and retrying");
        //}
      } else {

        String ret;
        do {
          ret = modem.setNetworkMode(54);
          delay(500);
        } while (!ret);

        String name = modem.getModemName();
        Serial.println("modem Name:" + name);
        String modemInfo = modem.getModemInfo();
        Serial.println("modem Info:" + modemInfo);

        //GESTIONE PIN
        if (modem.getSimStatus() != 3) {
          Serial.println("modem PIN Unlock");
          modem.simUnlock(PIN);
        }

        ModemUsable = true;
      }
    }

    if (ModemUsable) {
      Serial.println("GPRS Network");
      unsigned long timeout = millis();
      while (modem.waitForNetwork() && (millis() - timeout < 10)) {
        Serial.print(".");
        timeout = millis();
      }
      if (modem.isNetworkConnected()) {
        Serial.println("modem Network connected");
      } else {
        Serial.println("modem Network NOT connected");
      }

      Serial.println("GPRS Connection APN");
      if (!modem.gprsConnect(APN, LOGIN, PASSWORD)) {
        Serial.println("gprs fail");
        GPRSUsable = false;
      } else {
        Serial.println("gprs OK");
        if (modem.isGprsConnected()) {
          Serial.println("gprs INET OK");
          GPRSUsable = true;
          /*
            modem.sendAT(GF("+CNSMOD?"));
            if (modem.waitResponse(GF(GSM_NL "+CNSMOD:")) != 1) { }
            int nmodec = modem.stream.readStringUntil(',').toInt() != 0;
            int nmode = modem.stream.readStringUntil('\n').toInt();
            modem.waitResponse();
            Serial.println("Network Mode:" + nmode);

            IPAddress local = modem.localIP();
            Serial.println("Local IP:" + local);

            int csq = modem.getSignalQuality();
            Serial.println("Signal quality:" + csq);
            */
        } else {
          Serial.println("gprs INET KO");
          GPRSUsable = false;
        }
      }
    }
  }
#endif

/* CONNECTION LORA */
#ifdef LoraEnable
  Serial.println("LORA");
  esp_task_wdt_reset();  // Reset del watch-dog
    Serial.println("LORA Connection MODEM:" + (String)MODEM_LORA + 
                                    " BAND:" + (String)LORA_BANDWIDTH + 
                                    " POWER:" + (String)TX_OUTPUT_POWER;
    
    RadioEvents.RxDone = OnRxDone;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                               LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                               LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                               0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
#endif
}
String callURL(const String &Server = "api.geqo.it", const String &url = "/index.php?api=mytesttx", const String &body = "") {
  String response;
  bool NotTransmited = true;
  const char *host = Server.c_str();
  int port = 80;
  bool isHeader = true;
  String urlComplete = url;
  if (body != ""){
    String encodedString = base64::encode(body);
    urlComplete = url + "&chn=WIFI" + "&sensors=" + encodedString;
  }
   
  #ifdef EthEnable
    if (EthUsable && NotTransmited) {
      Serial.println("callURL ETH ");
      Serial.println((String) "GET " + urlComplete + " HTTP/1.1");
      Serial.println((String) "Host: " + host);
      if (e_client.connect(host, port)) {
        e_client.println((String) "GET " + urlComplete + " HTTP/1.1");
        e_client.println((String) "Host: " + host);
        e_client.println((String) "Connection: close");
        e_client.println();

        Serial.println("callURL ETH read respond:");
        unsigned long timeout = millis();
        while (e_client.connected() && (millis() - timeout < 10000L)) {
          while (e_client.available()) {
            if (isHeader) {
              String line = e_client.readStringUntil('\n');
              if (line == "\r") {
                isHeader = false;
              }
            } else {
              char c = e_client.read();
              response += c;
              timeout = millis();
            } 
          }
          NotTransmited = false;
        }
        if (e_client.connected()) {
          Serial.println("callURL ETH disconnecting from server.");
          e_client.stop();
        }
      } else {
        Serial.println("callURL ETH connection failed");
      }
    }
  #endif
  #ifdef WifiEnable
    if (WifiUsable && NotTransmited) {
      Serial.println("callURL WIFI ");
      Serial.println((String) "GET " + urlComplete + " HTTP/1.1");
      Serial.println((String) "Host: " + host);
      if (w_client.connect(host, port)) {
        w_client.println((String) "GET " + urlComplete + " HTTP/1.1");
        w_client.println((String) "Host: " + host);
        w_client.println((String) "Connection: close");
        w_client.println();

        Serial.println("callURL WIFI read respond:");
        unsigned long timeout = millis();
        while (w_client.connected() && (millis() - timeout < 10000L)) {
          while (w_client.available()) {
            if (isHeader) {
              String line = w_client.readStringUntil('\n');
              if (line == "\r") {
                isHeader = false;
              }
            } else {
              char c = w_client.read();
              response += c;
              timeout = millis();
            }
          }
          NotTransmited = false;
        }

        if (w_client.connected()) {
          Serial.println("callURL WIFI disconnecting from server.");
          w_client.stop();
        }
      } else {
        Serial.println("callURL WIFI connection failed");
      }
    }
  #endif
  #ifdef GprsEnable
    if (GPRSUsable && NotTransmited) {
      Serial.println("callURL GPRS ");
      Serial.println((String) "GET " + urlComplete + " HTTP/1.1");
      Serial.println((String) "Host: " + host);
      if (g_client.connect(host, port)) {
        g_client.println((String) "GET " + urlComplete + " HTTP/1.1");
        g_client.println((String) "Host: " + host);
        g_client.println((String) "Connection: close");
        g_client.println();

        Serial.println("callURL GPRS read respond:");
        unsigned long timeout = millis();
        while (g_client.connected() && (millis() - timeout < 10000L)) {
          while (g_client.available()) {
            if (isHeader) {
              String line = e_client.readStringUntil('\n');
              if (line == "\r") {
                isHeader = false;
              }
            } else {
              char c = e_client.read();
              response += c;
              timeout = millis();
            }
          }
          NotTransmited = false;
        }
        if (g_client.connected()) {
          Serial.println("callURL GPRS disconnecting from server.");
          g_client.stop();
        }
      } else {
        Serial.println("callURL GPRS connection failed");
      }
    }
  #endif
  #ifdef LoraEnable
    if (LORAUsable && NotTransmited) {
      Serial.println("callURL LORA ");
      Serial.println((String) "GET " + urlComplete + " HTTP/1.1");
      Serial.println((String) "Host: " + host);
      Radio.Send((uint8_t *)urlComplete, strlen(urlComplete));

      Serial.println("callURL LORA read respond:");
      unsigned long timeout = millis();
      while ((millis() - timeout < 10000L)) {
        if (lora_idle) {
          lora_idle = false;
          Radio.Rx(0);
        }
        Radio.IrqProcess();
        NotTransmited = false;
      }
    } else {
      Serial.println("callURL GPRS connection failed");
    }
  }
  #endif
  Serial.println("RAW received");
  Serial.println(response);
  return (response);
}
byte *getImageData(const String &url) {
  String response = callURL(url);
  byte *imageData = (byte *)malloc(response.length());
  if (!imageData) {
    Serial.println("Failed to allocate memory for image data");
    return nullptr;
  }
  for (unsigned int i = 0; i < response.length(); i++) {
    imageData[i] = (byte)response[i];
  }
  return imageData;
}

void MeasureTx() {
  esp_task_wdt_reset();  // Reset del watch-dog
  DynamicJsonDocument doc(1024);
  String jsonString;

  char measureChar[8];                  // Buffer big enough for 7-character float
  dtostrf(measure, 6, 2, measureChar);  // Leave room for too large numbers!

  doc["measure"] = measureChar;

  serializeJson(doc, jsonString);
  String response = callURL(geqo_url, (String) "/index.php?" + "api=" + geqo_api + "&" + "mac=" + chipId, jsonString);
  Serial.println("[MeasureTxHTTP]:");
  Serial.println(jsonString);

  if (!jsonString.isEmpty()) {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);

    if (error) {
      Serial.print("Scheduler Failed to parse JSON: ");
      Serial.println(error.c_str());
    }

    id = doc["ID"].as<String>();
    descrizione = doc["DESCRIZIONE"].as<String>();
    minimo = doc["MIN"].as<String>();
    massimo = doc["MAX"].as<String>();
    Serial.println(id);
    Serial.println(descrizione);
    Serial.println(minimo);
    Serial.println(massimo);
  }
}

void BarcodeTx() {
  esp_task_wdt_reset();  // Reset del watch-dog
  DynamicJsonDocument doc(1024);
  String jsonString;

  doc["barcode"] = lastBarcode;

  serializeJson(doc, jsonString);

  // Serial.println(jsonString);
  String response = callURL(geqo_url, (String) "/index.php?" + "api=" + geqo_api + "&" + "mac=" + chipId, jsonString);
  Serial.println("[BarcodeTxHTTP]:");
  Serial.println(jsonString);
  Serial.println(response);

  if (!response.isEmpty()) {
    DynamicJsonDocument doc(257600);
    DeserializationError error = deserializeJson(doc, response);

    if (error) {
      Serial.print("Scheduler Failed to parse JSON: ");
      Serial.println(error.c_str());
    }

    id = doc["ID"].as<String>();
    descrizione = doc["DESCRIZIONE"].as<String>();
    minimo = doc["MIN"].as<String>();
    massimo = doc["MAX"].as<String>();
    image_b64 = doc["IMAGE"].as<String>();

    Serial.print("Image: ");
    Serial.println(image_b64);  // null

    size_t decode_len = TD_BASE64.getDecodeLength(image_b64);
    // Serial.println(decode_len); // 3

    uint8_t image_decode[decode_len];
    TD_BASE64.decode(image_b64, image_decode);

    Serial.println(id);
    Serial.println(descrizione);
    Serial.println(minimo);
    Serial.println(massimo);
    Serial.println((char *)image_decode);


    spr.fillRect(0, LINEHEIGHT * 2, WIDTH, LINEHEIGHT * 2, TFT_BLACK);
    spr.setCursor(0, LINEHEIGHT * 2);
    spr.setTextSize(3);
    spr.println(descrizione + "\n");
    spr.setTextSize(4);
    spr.print(minimo);
    spr.print("<->");
    spr.println(massimo);

    spr.pushImage(296, 0, 280, 280, (uint16_t *)image_decode);
    // lcd_PushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
  }
}

/* WAKEUP  ---------------------------------------------------------------*/
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case 1:
      {
        Serial.println("Wakeup caused by external signal using RTC_IO");
        delay(2);
      }
      break;
    case 2:
      {
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        delay(2);
      }
      break;
    case 3:
      {
        Serial.println("Wakeup caused by timer");
        delay(2);
      }
      break;
    case 4:
      {
        Serial.println("Wakeup caused by touchpad");
        delay(2);
      }
      break;
    case 5:
      {
        Serial.println("Wakeup caused by ULP program");
        delay(2);
      }
      break;
    default:
      {
        Serial.println("Wakeup was not caused by deep sleep");
        delay(2);
      }
      break;
  }
}
void VextON(void) {
  Serial.println("VextON!");
  #ifdef HelTecEnable
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
  #endif
}
void VextOFF(void) {
  Serial.println("VextOFF!");
  #ifdef HelTecEnable
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, HIGH);
  #endif
}

/* MONITOR         -------------------------------------------------------*/
uint16_t colors[6] = { TFT_RED, TFT_GREEN, TFT_BLUE, TFT_YELLOW, TFT_CYAN, TFT_MAGENTA };

void drawArrayJpeg(const uint8_t arrayname[], uint32_t array_size, int xpos, int ypos) {

  int x = xpos;
  int y = ypos;

  JpegDec.decodeArray(arrayname, array_size);

  jpegInfo();  // Print information from the JPEG file (could comment this line out)

  renderJPEG(x, y);

  Serial.println("#########################");
}

void renderJPEG(int xpos, int ypos) {

  // retrieve information about the image
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // read each MCU block until there are no more
  while (JpegDec.read()) {

    // save a pointer to the image block
    pImg = JpegDec.pImage;

    // calculate where the image block should be drawn on the screen
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;  // Calculate coordinates of top left corner of current MCU
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // copy pixels into a contiguous block
    if (win_w != mcu_w) {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++) {
        p += mcu_w;
        for (int w = 0; w < win_w; w++) {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;

    tft.startWrite();

    // draw image MCU block only if it will fit on the screen
    if ((mcu_x + win_w) <= tft.width() && (mcu_y + win_h) <= tft.height()) {

      // Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
      tft.setAddrWindow(mcu_x, mcu_y, win_w, win_h);

      // Write all MCU pixels to the TFT window
      while (mcu_pixels--) {
        // Push each pixel to the TFT MCU area
        tft.pushColor(*pImg++);
      }

    } else if ((mcu_y + win_h) >= tft.height()) JpegDec.abort();  // Image has run off bottom of screen so abort decoding

    tft.endWrite();
  }

  // calculate how long it took to draw the image
  drawTime = millis() - drawTime;

  // print the results to the serial port
  Serial.print(F("Total render time was    : "));
  Serial.print(drawTime);
  Serial.println(F(" ms"));
  Serial.println(F(""));
}

void jpegInfo() {
  Serial.println(F("==============="));
  Serial.println(F("JPEG image info"));
  Serial.println(F("==============="));
  Serial.print(F("Width      :"));
  Serial.println(JpegDec.width);
  Serial.print(F("Height     :"));
  Serial.println(JpegDec.height);
  Serial.print(F("Components :"));
  Serial.println(JpegDec.comps);
  Serial.print(F("MCU / row  :"));
  Serial.println(JpegDec.MCUSPerRow);
  Serial.print(F("MCU / col  :"));
  Serial.println(JpegDec.MCUSPerCol);
  Serial.print(F("Scan type  :"));
  Serial.println(JpegDec.scanType);
  Serial.print(F("MCU width  :"));
  Serial.println(JpegDec.MCUWidth);
  Serial.print(F("MCU height :"));
  Serial.println(JpegDec.MCUHeight);
  Serial.println(F("==============="));
}

void MonitorInit() {
  Serial.println("MonitorInit!");
  #ifdef HelTecEnable
    display.init();
    display.clear();
    display.display();

    display.setContrast(255);

    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.clear();
    display.display();
    display.screenRotate(ANGLE_0_DEGREE);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 32 - 16 / 2, "GEQO");
    display.display();
  #endif
  #ifdef SPREnable
    rm67162_init();
    lcd_setRotation(1);
    spr.createSprite(WIDTH, HEIGHT);
    spr.fillScreen(TFT_BLACK);
    spr.setSwapBytes(1);
    spr.setTextColor(TFT_WHITE);

    spr.setCursor(0, 0);
    spr.setTextSize(4);
    spr.println("initialized..");
  #endif
}
void MonitorCls() {
  Serial.println("MonitorCls!");
  #ifdef HelTecEnable
    display.clear();
    display.display();
  #endif
  #ifdef SPREnable
    lcd_setRotation(1);
    spr.createSprite(WIDTH, HEIGHT);
    spr.fillScreen(TFT_BLACK);
    spr.setSwapBytes(1);
    spr.setTextColor(TFT_WHITE);
  #endif
}
void MonitorPrint(int16_t x, int16_t y, const String &text) {
  Serial.println("MonitorPrint!");
  #ifdef HelTecEnable
    display.drawString(x, y, text);
    display.display();
  #endif
  #ifdef SPREnable
    spr.fillSprite(TFT_BLACK);
    spr.setTextColor(colors[i], TFT_BLACK);
    spr.drawString(text, x, y, 4);
    lcd_PushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
  #endif
}
void MonitorImage() {
  Serial.println("MonitorImage!");
  #ifdef HelTecEnable
    display.drawString(x, y, text);
    display.display();
  #endif
  #ifdef SPREnable
    spr.pushImage(0, 0, WIDTH, HEIGHT, (uint16_t *)gImage_true_color);
    lcd_PushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
  #endif
}

/* UPDATE FIRMWARE -----------------------------------------------------------------*/
bool checkupdate(const String &Server = "api.geqo.it") {
  const char *host = Server.c_str();
  int port = 80;
  char buff[32];
  sprintf(buff, "/fota/rfid%d.bin", FWversion + 1);
  Serial.print("Firmware file ");
  Serial.println(buff);

  #ifdef EthEnable
    if (EthUsable) {
      Serial.println("Firmware ETH ");
      HttpClient http_client(e_client, host, port);
      http_client.get(buff);
      int statusCode = http_client.responseStatusCode();
      Serial.print("Firmware status code: ");
      Serial.println(statusCode);
      if (statusCode != 200) {
        http_client.stop();
        return false;
      }

      long length = http_client.contentLength();
      if (length == HttpClient::kNoContentLengthHeader) {
        http_client.stop();
        Serial.println("Firmware Error Content-length header");
        return false;
      }
      Serial.print("Firmware update file size ");
      Serial.println(length);

      if (!InternalStorage.open(length)) {
        http_client.stop();
        Serial.println("Firmware Error not enough space");
        return false;
      }
      byte b;
      while (length > 0) {
        if (!http_client.readBytes(&b, 1))
          break;
        InternalStorage.write(b);
        length--;
      }
      InternalStorage.close();
      http_client.stop();
      if (length > 0) {
        Serial.println("Firmware Error Timeout");
        return false;
      }
      Serial.println("Firmware UPDATED");
      Serial.flush();
      InternalStorage.apply();
      return true;
    }
  #endif

  #ifdef WifiEnable
    if (WifiUsable) {
      Serial.println("FIRMWARE WIFI ");
      HttpClient http_client(w_client, host, port);
      http_client.get(buff);
      int statusCode = http_client.responseStatusCode();
      Serial.print("Firmware status code: ");
      Serial.println(statusCode);
      if (statusCode != 200) {
        http_client.stop();
        return false;
      }

      long length = http_client.contentLength();
      if (length == HttpClient::kNoContentLengthHeader) {
        http_client.stop();
        Serial.println("Firmware Error Content-length header");
        return false;
      }
      Serial.print("Firmware update file size ");
      Serial.println(length);

      if (!InternalStorage.open(length)) {
        http_client.stop();
        Serial.println("Firmware Error not enough space");
        return false;
      }
      byte b;
      while (length > 0) {
        if (!http_client.readBytes(&b, 1))
          break;
        InternalStorage.write(b);
        length--;
      }
      InternalStorage.close();
      http_client.stop();
      if (length > 0) {
        Serial.println("Firmware Error Timeout");
        return false;
      }

      Serial.println("Firmware UPDATED");
      Serial.flush();
      InternalStorage.apply();
      return true;
    }
  #endif

  #ifdef GprsEnable
    if (GPRSUsable) {
      Serial.println("FIRMWARE GPRS ");
      HttpClient http_client(g_client, host, port);
      http_client.get(buff);
      int statusCode = http_client.responseStatusCode();
      Serial.print("Firmware status code: ");
      Serial.println(statusCode);
      if (statusCode != 200) {
        http_client.stop();
        return false;
      }

      long length = http_client.contentLength();
      if (length == HttpClient::kNoContentLengthHeader) {
        http_client.stop();
        Serial.println("Firmware Error Content-length header");
        return false;
      }
      Serial.print("Firmware update file size ");
      Serial.println(length);

      if (!InternalStorage.open(length)) {
        http_client.stop();
        Serial.println("Firmware Error not enough space");
        return false;
      }
      byte b;
      while (length > 0) {
        if (!http_client.readBytes(&b, 1))
          break;
        InternalStorage.write(b);
        length--;
      }
      InternalStorage.close();
      http_client.stop();
      if (length > 0) {
        Serial.println("Firmware Error Timeout");
        return false;
      }
      Serial.println("Firmware UPDATED");
      Serial.flush();
      InternalStorage.apply();
      return true;
    }
  #endif


  return false;
}

/* SETUP ---------------------------------------------------------------------*/
void setup() {
  Serial.begin(9600);
  print_wakeup_reason();
  //delay(5000);
  /*  DISPLAY -----------------------------------------------------------------*/
  MonitorInit();
  //VextON();
  //delay(5000);

  Wire.begin();

  /* INIZIALIZZAZIONE DEL WATCH-DOG-------------------------------------------*/
  esp_task_wdt_init(WDT_TIMEOUT, true);  // Abilita il riavvio su fermata prolungata
  esp_task_wdt_add(NULL);                // Thread da aggiungere alla finestra del watch-dog [NULL = loop principale]

  Serial.println("Hello!");
  uint64_t EfuseMac = ESP.getEfuseMac();
  chipId = mac2String((byte *)&EfuseMac);
  Serial.print("ESP32ChipID");
  Serial.println(chipId);

  /* Inizializzazione pin----------------------------------------------------- */
  pinMode(BOOTPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);

  #ifdef SCANPIN
    pinMode(TRIGGERPIN, OUTPUT);
    pinMode(ZEROPIN, OUTPUT);
  #endif
  #ifdef CALIPERPIN
    pinMode(CALIPERPIN, INPUT);
  #endif

  /* INIZIALIZZAZIONE BARCODE SCANNER-----------------------------------------*/
  #ifdef SCANPIN
    pinMode(SCANPIN, INPUT);
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    digitalWrite(TRIGGERPIN, triggerStatus);
  #endif

  /* INIZIALIZZAZIONE CALIPER -------------------------------------------------*/
  #ifdef CALIPERPIN
  #endif

  //get MAC ADDRESS to Ethernet
  #ifdef EthEnable
    uint8_t macRaw[6];
    esp_read_mac(macRaw, ESP_MAC_WIFI_STA);
    for (int i = 0; i < 6; i++) {
      mac[i] = macRaw[i];
    }
  #endif
    /* EPROM GEQO --------------------------------------------------------------*/
    Serial.println("EPROM");
    settings.begin("geqo", false);
    if (settings.getString("geqo_url", "") == "") {
      settings.putString("geqo_url", geqo_url);
      settings.putString("geqo_machine", geqo_machine);
      settings.putString("geqo_user", geqo_user);
      settings.putString("geqo_pwd", geqo_pwd);
      settings.putString("geqo_dbname", geqo_dbname);
      settings.putString("geqo_align", geqo_align);
      settings.putString("geqo_api", geqo_api);
      settings.putString("geqo_zone", geqo_zone);
      settings.putString("iotname", iotName);
    }
    geqo_url = settings.getString("geqo_url", "");
    geqo_machine = settings.getString("geqo_machine", "");
    geqo_user = settings.getString("geqo_user", "");
    geqo_pwd = settings.getString("geqo_pwd", "");
    geqo_dbname = settings.getString("geqo_dbname", "");
    geqo_align = settings.getString("geqo_align", "");
    geqo_api = settings.getString("geqo_api", "");
    geqo_zone = settings.getString("geqo_zone", "");
    iotName = settings.getString("iotname", "");
    settings.end();

  #ifdef GprsEnable
    settings.begin("gprs_config", false);
    if (settings.getString("GPRS_APN", "") == "") {
      settings.putString("GPRS_PIN", GPRS_PIN);
      settings.putString("GPRS_APN", GPRS_APN);
      settings.putString("GPRS_LOGIN", GPRS_LOGIN);
      settings.putString("GPRS_PASSWORD", GPRS_PASSWORD);
    }
    GPRS_PIN = settings.getString("GPRS_PIN", "");
    GPRS_APN = settings.getString("GPRS_APN", "");
    GPRS_LOGIN = settings.getString("GPRS_LOGIN", "");
    GPRS_PASSWORD = settings.getString("GPRS_PASSWORD", "");
    settings.end();
  #endif

  // /* SCHEDULER LOOP ----------------------------------------------------------*/
  // Serial.println("Scheduler:");
  // scheduler.eprom = true;  //SAVE in EPROM SCHEDULER
  // scheduler.begin();
  // //scheduler.addTask(0, 8, 0, 19, 30, 30, 5, "PMP");
  // //scheduler.addTask(1, 8, 0, 18, 30, 30, 5, "PMP");
  // //scheduler.addTask(2, 8, 0, 17, 30, 30, 5, "PMP");

  /* CONNECT ONLINE  ---------------------------------------------------------*/
  Serial.println("ONLINE");
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("connectToInet:");
  connectToInet();

  /* TIME SYNC RTC -----------------------------------------------------------*/
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("TIME SYNC:");
  String response = callURL(geqo_url, "/index.php?api=" + geqo_api + "&" + "mac=" + chipId + "&" + "hb=1");
  if (!response.isEmpty()) {
    Serial.println("TIME HTTP:" + response);
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, response);
    if (error) {
      Serial.println((String) "TIME Scheduler Failed to parse JSON: " + error.c_str());
      //date time align from ext rtc
      syncTimeFromUTC(geqo_zone, 0);
    } else {
      //timing update align
      Serial.println("convert json to var");
      settings.begin("geqo", false);

      int uValue = doc["U"].as<int>();
      if (uValue > 3600) uValue = 3600;
      if (uValue < 10) uValue = 10;

      geqo_align = String(uValue);
      Serial.println(geqo_align);
      settings.putString("geqo_align", geqo_align);

      int dValue = doc["D"].as<int>();
      geqo_zone = doc["Z"].as<String>();
      Serial.println(geqo_zone);
      settings.putString("geqo_zone", geqo_zone);

      geqo_machine = doc["DD"].as<String>();
      Serial.println(geqo_machine);
      settings.putString("geqo_machine", geqo_machine);

      settings.end();

      syncTimeFromUTC(geqo_zone, dValue);
    }
  } else {
    Serial.println("TIME HTTP: NO RESPONSE");
    syncTimeFromUTC(geqo_zone, 0);
  }

  /* REMOTE FIRMWARE UPDATE  -------------------------------------------------*/
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("FIRMWARE:");
  bool responseUpd = checkupdate(geqo_url);

  /* RETRIVE GEQO SCHEDULER ONLINE ---------------------------------------------*/
  #ifdef SCHEnable
    Serial.println("SCHEDULER:");
    esp_task_wdt_reset();  // Reset del watch-dog
    SchedulerRx();
  #endif

    /* HTTP  SERVER ------------------------------------------------------------------*/
    Serial.println("WEB SERVER");
    esp_task_wdt_reset();  // Reset del watch-dog
    app.use("/", &handle_root);
    app.use("/style.css", &handle_css);
    app.use("/manifest.json", &handle_manifest);
    app.notFound(&notFound);
    app.use("/srvconfig", &handle_SrvConfig);
    app.use("/reconnect", &handle_reconnect);
    app.use("/reset", &handle_reset);
    app.use("/upload", &handle_upload);
    app.use("/update", &handle_update);

    app.use("/sensors", &handle_Sensors);
    app.use("/sensorsrx", &handle_SensorsRx);

  #ifdef SCHEnable
    app.use("/schedulerconfig", &handle_SchedulerConfig);
    app.post("/schedulerconfigset", &handle_SchedulerConfigSet);
    app.use("/schedulerrx", &handle_SchedulerRx);
  #endif

  #ifdef GPRSEnable
    app.use("/gprsconfig", &handle_GprsConfig);
  #endif

  #ifdef WIFIEnable
    app.use("/wificonfig", &handle_WiFiConfig);
    WifServer.begin();
  #endif

  #ifdef ETHEnable
    app.use("/ethconfig", &handle_EthConfig);
    EthServer.begin();
  #endif


  /* RETRIVE IMAGE  ONLINE ----------------------------------------------------*/
  /* Get image data from URL  */
  String responseImg = callURL(geqo_url, "/ingr.bmp","");
  
  uint16_t *imageData = (uint16_t *)malloc(responseImg.length());
  uint8_t bmpHeader[54];
  for (unsigned int i = 0; i < 54; i++) {
    bmpHeader[i] = (byte)responseImg[i];
    Serial.print(responseImg[i]);
  }
  int32_t bmpWidth = *(int32_t *)&bmpHeader[18];
  int32_t bmpHeight = *(int32_t *)&bmpHeader[22];
  Serial.println();
  Serial.println(bmpWidth);
  Serial.println(bmpHeight);
  Serial.println(bmpHeader[0]);
  Serial.println(bmpHeader[1]);
  Serial.println(bmpHeader[30]);

  // Load BMP uncompressed
  spr.fillSprite(TFT_BLACK);
  int padding = (4 - (bmpWidth * 3) % 4) % 4;
  spr.createSprite(bmpWidth, bmpHeight);
  uint8_t *lineBuffer = (uint8_t *)malloc(bmpWidth * 3);
  int offsetRow = 54; //54
  for (int y = 0; y < bmpHeight; y++) {
    for (unsigned int i = 0; i < bmpWidth*3; i++) {
      lineBuffer[i] = (byte)responseImg[i+offsetRow];
    }
    for (int x = 0; x < bmpWidth; x++) {
      uint16_t color = spr.color565(lineBuffer[x * 3 + 2], lineBuffer[x * 3 + 1], lineBuffer[x * 3]);
      spr.drawPixel(x, bmpHeight - y - 1, color);  
    }
    offsetRow = (y*bmpWidth * 3) ;
    offsetRow = (y*bmpWidth * 3) + padding;
  }
  free(lineBuffer);
  spr.pushSprite(0, 0);
  lcd_PushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());


  /*
    spr.setRotation(2);  // portrait
    spr.fillScreen(random(0xFFFF));
    int x = (spr.width()  - 300) / 2 - 1;
    int y = (spr.height() - 300) / 2 - 1;
    drawArrayJpeg(imageData, sizeof(imageData), x, y); // Draw a jpeg image stored in memory at x,y
    delay(2000);

    //drawJpeg(imageData,responseImg.length());

  */
  /* START LOOP2 -------------------------------------------------------------
    Serial.println("START SECOND CORE");
    xTaskCreatePinnedToCore(
                            loop2,     //Task function.
                            "loop2",   // name of task. 
                            10000,     // Stack size of task 
                            NULL,      // parameter of the task 
                            1,         // priority of the task 
                            &Task2,    // Task handle to keep track of created task 
                            0);        // pin task to core 1 
  */

  PREV_MILLIS_CONN = millis();
  Serial.println("OK ALL started");
}


/* LOOP MAIN  ----------------------------------------------------------------*/

void loop() {
}

void loop2(void *pvParameters) {
  DynamicJsonDocument doc(2048);
  while (true) {
#ifdef AQBPIN
    AirQuality(doc);
#endif

#ifdef ANEPIN
    Anemometer(doc);
#endif

#ifdef LEVEL1PIN
    LevelTank(doc);
#endif

    jsonString = "";
    serializeJson(doc, jsonString);
    delay(1000);
  }
}

void loop3() {

  esp_task_wdt_reset();  // Reset del watch-dog

  /* CONNECT ONLINE SCHEDULER UPDATE -----------------------------------------*/
  if (geqo_align.toInt() > 3600) {
    geqo_align = "3600";
  }
  if (geqo_align.toInt() < 10) {
    geqo_align = "10";
  }
  if (geqo_align.toInt() > 0) {
    if (millis() - PREV_MILLIS_CONN > (geqo_align.toInt() * 1000)) {
      PREV_MILLIS_CONN = millis();
#ifdef SCHEnable
      SchedulerRx();
#endif
    }
  }

/* GESTIONE WIFI CHIAMATE SITO WEB */
#ifdef WifiEnable
  WiFiClient WifiClient = WifServer.available();
  if (WifiClient.connected()) {
    app.process(&WifiClient);
    WifiClient.stop();
  }
#endif

/* GESTIONE ETH CHIAMATE SITO WEB */
#ifdef EthEnable
  if (!EthUsable) {
    if (Ethernet.linkStatus() == LinkON) {
      Serial.println("Ethernet Reconnected");
      connectToInet();
    }
  } else {
    if (Ethernet.linkStatus() == LinkOFF) {
      EthUsable = false;
      Serial.println("Ethernet Disconnected");
    } else {
      EthernetClient EthClient = EthServer.available();
      if (EthClient.connected()) {
        app.process(&EthClient);
        EthClient.stop();
      }
    }
  }
#endif

  dnsServer.processNextRequest();

//GESTIONE SCHEDULER
#ifdef SCHEnable
  scheduler.run();
#endif

// // EVENTI OGNI SECONDO
// if ((millis() - PREV_MILLIS_SECOND) > 1000) {
//   PREV_MILLIS_SECOND = millis();
// }

/* Gestione BOOTPIN --------------------------------------------------*/
// invio della misura [possibile solamente se è stato scansionato un codice a barre]
#ifdef BOOTPIN
  if ((digitalRead(BOOTPIN) == 0) && (bootPinStatus == 1)) {
    PREV_MILLIS_BTN = millis();
    Serial.println("Measure premuto");
    bootPinStatus = 0;
  }

  if ((digitalRead(BOOTPIN) == 1) && (bootPinStatus == 0) && (millis() - PREV_MILLIS_BTN) > PUSHTIME) {
#ifdef CALIPERPIN
    zeroMeasure();
#endif
    PREV_MILLIS_BTN = millis();
    bootPinStatus = 1;
    Serial.println("BOOTPIN SET");
  }
  if ((digitalRead(BOOTPIN) == 1) && (bootPinStatus == 0) && (millis() - PREV_MILLIS_BTN) < PUSHTIME && lastBarcode != "") {
#ifdef CALIPERPIN
    MeasureTx();
#endif
    PREV_MILLIS_BTN = millis();
    bootPinStatus = 1;
    Serial.println("BOOTPIN REST");
  }

  // if ((digitalRead(BOOTPIN) == 1) && (bootPinStatus == 0)) {
  //   if ((millis() - PREV_MILLIS_BTN) > 3000) {
  //     //Reset WIFI se premuto pulsante per più di 5 secondi, DA FARE
  //     /*Serial.println("Erase settings and restart ...");
  //     delay(1000);
  //     ConnectionWifi();*/
  //     ESP.restart();
  //   }
  //   Serial.println("Pulsante rilasciato");
  //   buttonState = 1;
  //   PREV_MILLIS_BTN = millis();
  // }
#endif

/* Gestione SCANPIN --------------------------------------------------*/
// scansione e invio codice a barre
#ifdef SCANPIN
  if ((digitalRead(SCANPIN) == 0) && (scanPinStatus == 1)) {
    // PREV_MILLIS_BTN = millis();
    Serial.println("Scansione ...");
    scanCode();
    scanPinStatus = 0;
  }

  if ((digitalRead(SCANPIN) == 1) && (scanPinStatus == 0)) {
    scanPinStatus = 1;
    // Serial.println("aaaaa");
  }
#endif

/* Gestione CLOCKPIN ------------------------------------------------*/
#ifdef CALIPERPIN
  while (digitalRead(CLOCKPIN) == HIGH) {}  //if clock is LOW wait until it turns to HIGH
  tempmicros = micros();
  while (digitalRead(CLOCKPIN) == LOW) {}  //wait for the end of the HIGH pulse
  if ((micros() - tempmicros) > 500) {     //if the HIGH pulse was longer than 500 micros we are at the start of a new bit sequence
    readCaliper();                         //decode the bit sequence
  }
#endif

  // //cose da fare mentre il pulsante è premuto
  // if (buttonState == 0) {
  // }

  esp_task_wdt_reset();  // Reset del watch-dog
}
