#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <WebSocketsClient.h>     //https://github.com/Links2004/arduinoWebSockets
#include <Hash.h>

#define LDR_PIN A0   //LDR pin
#define BUTTON_PIN 4 //Button
#define RLED_PIN 15  //red LED PIN
#define GLED_PIN 12  //green LED PIN
#define BLED_PIN 13  //blue LED PIN

#define WS_NONE 9999
#define WS_GET_RID 100

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

/*==============================================================================
  WIFI Configuration Variables - Change these variables to your own settings.
  =============================================================================*/

const int SENSOR_INTERVAL = 100;
const int WSLINK_INTERVAL = 3000;//milliseconds period for reporting to Exosite.com

// Use these variables to customize what dataports are read and written to.
String readString = "rled";
String readString2 = "&gled";
String readString3 = "&bled";
String writeString = "button=";
String writeString2 = "&ldr=";
String returnString;
int ldrValue, ldrDuration;

unsigned int localPort = 2390; // local port to listen for UDP packets

char cik[41];
char ssid[32];
char password[32];
char domain[30] = "m2.exosite.com";
char productId[16];
WebSocketsClient webSocket;
bool wsAuthCheck = false;
bool wsAuthStatus = false;
char rid[41];
int wsProcess = WS_NONE;
int LastButtonStatus = 1;
int LastLdrStatus = 0;

/*==============================================================================
  End of Configuration Variables
  =============================================================================*/
long unsigned int prevSendTime = 0;
long unsigned int prevSensorTime = 0;
int exosite_comm_errors = 0;
int comm_errors = 0;


//NTP
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48;
WiFiUDP udp;
byte packetBuffer[ NTP_PACKET_SIZE];
unsigned long TimeStamp = 1466162486;

/*==============================================================================
  setup

  Arduino setup function.
  =============================================================================*/
void setup()
{
  uart_div_modify(0, UART_CLK_FREQ / 115200);
  Serial.begin(115200);
  Serial.println();
  //clean FS, for testing
  //    SPIFFS.format();delay(100);

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(cik, json["cik"]);
          strcpy(domain, json["domain"]);
          strcpy(productId, json["productId"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }




  //SET UP IO PINS
  pinMode(LDR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(RLED_PIN, OUTPUT);
  pinMode(GLED_PIN, OUTPUT);
  pinMode(BLED_PIN, OUTPUT);

  Serial.println("\nsetup: Exosite Murano Platform Example App");

  WiFiManagerParameter custom_cik("cik", "cik", cik, 41);
  WiFiManagerParameter custom_product_id("productId", "productId", productId, 16);
  WiFiManagerParameter custom_domain("domain", "domain", domain, 30);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_cik);
  wifiManager.addParameter(&custom_product_id);
  wifiManager.addParameter(&custom_domain);

  //reset settings - for testing
//  wifiManager.resetSettings();
  //  wifiManager.setTimeout(120);

  String ssid = "MuranoDevice" + String(ESP.getChipId());
  Serial.println("Please connected to ssid:" + ssid);
  if (!wifiManager.autoConnect(ssid.c_str(), NULL)) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  Serial.println("connected...yeey :)");

  Serial.print("local ip:");
  Serial.println(WiFi.localIP());
  strcpy(cik, custom_cik.getValue());
  strcpy(productId, custom_product_id.getValue());
  strcpy(domain, custom_domain.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    saveConfig();
  }
  udp.begin(localPort);

  webSocket.begin(String(String(productId) + "." + domain), 80, "/ws");
  webSocket.onEvent(webSocketEvent);
  prevSensorTime = millis();
}

void saveConfig()
{
  Serial.println("saving config");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["cik"] = cik;
  json["domain"] = domain;
  json["productId"] = productId;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
  Serial.println("");
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t lenght) {
  char msg[200];
  char msg2[200];
  int values;
  String Msgs;
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WSc] Disconnected!\n");
      wsAuthStatus = false;
      break;
    case WStype_CONNECTED:
      {
        Serial.printf("[WSc] Connected to url: %s\n",  payload);

        // send message to server when Connected
        char Auth[80];
        sprintf(Auth, "{\"auth\":{\"cik\":\"%s\"}}", cik);
        webSocket.sendTXT(Auth);
        wsAuthCheck = true;
        getNTPTime();
      }
      break;
    case WStype_TEXT:
      if (wsAuthCheck)
      {
        sprintf(msg, "%s", payload);
        if (String(msg).equalsIgnoreCase("{\"status\": \"ok\"}"))
        {
          Serial.println("[WSc] Auth Sucess.");
          wsAuthStatus = true;
          if (String(rid).length() == 0) {
            wsProcess = WS_GET_RID;
            Serial.println("[WSc] Get RID first.");
          }
        }
        wsAuthCheck = false;
      } else {
        if (wsAuthStatus) {
          sprintf(msg, "%s", payload);
          StaticJsonBuffer<300> jsonBuffer;
          JsonArray& root = jsonBuffer.parseArray(msg);
          switch (wsProcess) {
            case WS_NONE:
              Serial.printf("[WSc] Get response: %s\n", payload);
              if (!root.success()) {
                Serial.println("[WSc] NONE parseObject() failed");
                return;
              }
              sprintf(msg2, "%s", payload);
              Msgs = String(msg2);
              if (Msgs.length() > 20 && Msgs.substring(8, 12) == "rled") {
                values = root[0]["result"][1];
                analogWrite(RLED_PIN, map(values, 0, 255, 0, 1024));
                Serial.printf("R:%d \n", values);
              } else if (Msgs.length() > 30 && Msgs.substring(8, 12) == "gled") {
                values = root[0]["result"][1];
                analogWrite(GLED_PIN, map(values, 0, 255, 0, 1024));
                Serial.printf("G:%d \n", values);
              } else if (Msgs.length() > 30 && Msgs.substring(8, 12) == "bled") {
                values = root[0]["result"][1];
                analogWrite(BLED_PIN, map(values, 0, 255, 0, 1024));
                Serial.printf("B:%d \n", values);
              }
              break;
            case WS_GET_RID:
              //get rid
              if (!root.success()) {
                Serial.println("[WSc] parseObject() failed");
                return;
              }
              strcpy(rid, root[0]["result"]);
              Serial.printf("[WSc] RID: %s \n", rid);
              String subscribe1 = "{\"calls\":[{\"id\":\"rled\",\"procedure\":\"subscribe\",\"arguments\":[{\"alias\":\"rled\",\"rid\":\"" + String(rid) + "\"},{\"since\":" + String(TimeStamp) + "}]}]}";
              String subscribe2 = "{\"calls\":[{\"id\":\"gled\",\"procedure\":\"subscribe\",\"arguments\":[{\"alias\":\"gled\",\"rid\":\"" + String(rid) + "\"},{\"since\":" + String(TimeStamp) + "}]}]}";
              String subscribe3 = "{\"calls\":[{\"id\":\"bled\",\"procedure\":\"subscribe\",\"arguments\":[{\"alias\":\"bled\",\"rid\":\"" + String(rid) + "\"},{\"since\":" + String(TimeStamp) + "}]}]}";
              webSocket.sendTXT(subscribe1.c_str());
              Serial.println("SEND:" + subscribe1);
              webSocket.sendTXT(subscribe2.c_str());
              Serial.println("SEND:" + subscribe2);
              webSocket.sendTXT(subscribe3.c_str());
              Serial.println("SEND:" + subscribe3);
              wsProcess = WS_NONE;
              break;
          }
        }
      }
      break;
    case WStype_BIN:
      Serial.printf("[WSc] get binary lenght: %u\n", lenght);
      hexdump(payload, lenght);

      // send data to server
      // webSocket.sendBIN(payload, lenght);
      break;
  }
}

unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
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
}

void getNTPTime()
{
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
  } else {
    udp.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);
    TimeStamp = epoch;
  }
}

/*==============================================================================
  loop

  Arduino loop function.
  =============================================================================*/
void loop()
{
  int8_t index = 0;
  int8_t lastIndex = -1;
  String Smsg;
  bool LdrSend = false;
  bool buttonSend = false;
  webSocket.loop();

  if (prevSensorTime + SENSOR_INTERVAL < millis() || millis() < prevSensorTime)
  {
    Serial.print("LDR: ");
    ldrValue = analogRead(LDR_PIN);
    ldrDuration = map(ldrValue, 0, 1023, 1, 100);
    Serial.println(ldrDuration);
    prevSensorTime = millis();
    if (wsAuthStatus)
    {
      switch (wsProcess) {
        case WS_NONE:
          if (LastLdrStatus != ldrDuration) {
            Smsg = String("{\"calls\": [{\"id\": 9999, \"procedure\": \"write\", \"arguments\": [{\"alias\": \"ldr\"}, \"" + String(ldrDuration, DEC) + "\", {}]}]}");
            Serial.println("SEND:" + Smsg);
            webSocket.sendTXT(Smsg.c_str());
            LastLdrStatus = ldrDuration;
            LdrSend = true;
          }
          if (LastButtonStatus != digitalRead(BUTTON_PIN))
          {
            Smsg = String("{\"calls\": [{\"id\": 9998, \"procedure\": \"write\", \"arguments\": [{\"alias\": \"button\"}, \"" + String(digitalRead(BUTTON_PIN)) + "\", {}]}]}");
            Serial.println("SEND:" + Smsg);
            webSocket.sendTXT(Smsg.c_str());
            LastButtonStatus = digitalRead(BUTTON_PIN);
            buttonSend = true;
          }
          break;
      }
    }
  }

  if (prevSendTime + WSLINK_INTERVAL < millis() || millis() < prevSendTime ) {
    if (TimeStamp == 0 ) {
      getNTPTime();
    }
    if (wsAuthStatus)
    {
      switch (wsProcess) {
        case WS_NONE:
          if (!LdrSend) {
            Smsg = String("{\"calls\": [{\"id\": 9999, \"procedure\": \"write\", \"arguments\": [{\"alias\": \"ldr\"}, \"" + String(ldrDuration, DEC) + "\", {}]}]}");
            Serial.println("SEND:" + Smsg);
            webSocket.sendTXT(Smsg.c_str());
          }
          if (!buttonSend) {
            Smsg = String("{\"calls\": [{\"id\": 9998, \"procedure\": \"write\", \"arguments\": [{\"alias\": \"button\"}, \"" + String(digitalRead(BUTTON_PIN)) + "\", {}]}]}");
            Serial.println("SEND:" + Smsg);
            webSocket.sendTXT(Smsg.c_str());
          }
          break;
        case WS_GET_RID:
          Smsg = String("{\"calls\": [{\"id\": 0, \"procedure\": \"lookup\", \"arguments\": [\"alias\", \"\"]}]}");
          Serial.println("SEND:" + Smsg);
          webSocket.sendTXT(Smsg.c_str());
          break;
      }
    }
    prevSendTime = millis();
  }
}

