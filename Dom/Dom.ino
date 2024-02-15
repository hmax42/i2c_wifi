/*
   M5Atom-Hydra

   Hardware-Compatibility

   ^----------------------------------------------------------------------------------------------^
   |  Hardware                     | Dom | Sub | I2C | ESP-now | Notes                            |
   |  Atom Lite                    |  x  |  x  |  x  |    ?    |                                  |
   |  Atom (Lite) Matrix           |  x  |  x! |  x  |    ?    |                                  |
   |  Atom (Lite) Echo             |  x  |  x  |  x  |    x    | i2s conflicts spi & gps serial   |
   |  Atom S3 (Lite) Oled          |  x! |  ?  |  x  |    ?    |                                  |
   |  Atom S3 Lite                 |  x  |  ?  |  x  |    ?    | ch5,7,8 nothing found            |
   |  Stamp (Mate)                 |  ?  |  x  |  -  |    x    | i2c slave error                  |
   |  Stamp (C3)                   |  ?  |  ?  |  -  |    x    | i2c slave error                  |
   |  Stamp (S3)                   |  ?  |  ?  |  -  |    ?    | untested (i2c slave error)       |
   |                               |     |     |     |         |                                  |
   °----------------------------------------------------------------------------------------------°


*/


//#define COMM_I2C
#define COMM_NOW


//#define DomServer
//use esp32 (2.0.13), 80, dio 80, 8mb
//#define S3OLED

#include <WiFi.h>
#ifdef DomServer
#include <WebServer.h>
#endif

#ifdef COMM_I2C
#include <Wire.h>
#endif

#ifdef COMM_NOW
#include <esp_now.h>
//how often to send the msg to subs to start scanning (in case a sub reboots or is rebooted)
//1 min = 60000
#define BROADCASTINTERVALL 60000
#endif

#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#ifdef S3OLED
#define SUB_SDA 2
#define SUB_SCL 1
#define GPS_RX 5
#define SD_CLK 7
#define SD_MISO 8
#define SD_MOSI 6
#define BTN 41
#define GPSSERIAL Serial2
#include <M5AtomS3.h>
int const blinkSize = 10;
int const centerText = 3;
int fg = WHITE;
int bg = BLACK;
bool fileInit = false;
#else
#define SUB_SDA 26
#define SUB_SCL 32
#define GPS_RX 22
#define SD_CLK 23
#define SD_MISO 33
#define SD_MOSI 19
#define BTN 39
#include <FastLED.h>
#define GPSSERIAL Serial1
const int LED_PIN = 27;
CRGB led;
#endif

TinyGPSPlus gps;
#ifdef DomServer
WebServer server(80);
#endif
File dataFile;
String fileName;

#ifdef DomServer
const char* ssid = "ESP32_Dom_Network";
const char* password = "12345678";
#endif
#ifdef COMM_I2C
const int i2c_slave_address = 0x55;
#define TCAADDR 0x70
#endif
#define NUM_PORTS 6
#ifdef S3OLED
bool oled = true;
#endif

struct NetworkInfo {
  char ssid[32];
  char bssid[18];
  int32_t rssi;
  char security[20];
  uint8_t channel;
  char type;
  int boardId;
};

NetworkInfo receivedNetworks[NUM_PORTS];

#ifdef DomServer
int totalNetworksSent[NUM_PORTS] = { 0 };
#endif
#ifdef S3OLED
// 0 is BLE
int countNetworks[15] = {0};
#endif


#ifdef COMM_NOW
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;
long lastBroadcastMillis = -1;
NetworkInfo myData;

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  int node = myData.boardId;
  Serial.println("Received " + String(myData.ssid));
#ifdef DomServer
  totalNetworksSent[node]++;
#endif
#ifdef S3OLED
  countNetworks[myData.channel]++;
#endif
  logData(myData, node);
}

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
#endif

void setup() {
#ifdef S3OLED
  auto cfg = M5.config();
  AtomS3.begin(cfg);
  //  AtomS3.Display.setTextFont(&fonts::TomThumb);
  //  AtomS3.Display.setTextSize(2);
  AtomS3.Display.setTextFont(&fonts::FreeSerif9pt7b);
  AtomS3.Display.setTextSize(1);
  AtomS3.Display.fillScreen(bg);
  AtomS3.Display.setTextColor(fg);
#else
  FastLED.addLeds<WS2812, LED_PIN, GRB>(&led, 1);
  led = CRGB::Black;
  FastLED.show();
  Serial.begin(115200);
#endif
#ifdef S3OLED
  drawGPSStatus();
  drawFileStatus();
#endif
#ifdef DomServer
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("AP IP Address: " + IP.toString());
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
#endif
#ifdef COMM_I2C
  Wire.begin(SUB_SDA , SUB_SCL);  // SDA, SCL
  Serial.println("[MASTER] I2C Master Initialized");
#endif

  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, -1);  // Initialize SPI for SD card
  if (!SD.begin()) {
    Serial.println("SD Card initialization failed!");
    unsigned long startMillis = millis();
    const unsigned long blinkInterval = 500;
    while (millis() - startMillis < 5000) {  // Continue blinking for 5 seconds
#ifdef S3OLED
      processButton();
#endif
      if (((millis() - startMillis) / blinkInterval) % 2  == 1 ) {
        blinkLEDRed();
      }
    }
    Serial.println("Starting without SD Card, no GPS fix and no proper file initialization.");
    return;
  }
  Serial.println("SD Card initialized.");
  GPSSERIAL.begin(9600, SERIAL_8N1, GPS_RX, -1);  // GPS Serial
  waitForGPSFix();

  initializeFile();
  Serial.println("File created.");


#ifdef COMM_NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  Serial.println("[MASTER] ESP-NOW initialized");

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // tell sub to start scanning
  sendSubMsg();
#endif
}

#ifdef COMM_NOW
uint8_t dummy = 0;

void  sendSubMsg() {
  esp_now_send(broadcastAddress, (uint8_t*)&dummy, sizeof(dummy));
  lastBroadcastMillis = millis();
}
#endif

#ifdef S3OLED
long displayUpdateDelay = 15000;
long lastDisplayUpdate = -15000;

void processButton() {
  AtomS3.update();

  if (AtomS3.BtnA.wasReleased()) {
    oled = !oled;
    if (!oled) {
      AtomS3.Display.clearDisplay();
    }
  }
}

void drawGPSStatus() {
  if (!oled) return;
  drawStatus(gps.location.isValid(), 'G', 3, MAGENTA);
}

void drawFileStatus() {
  if (!oled) return;
  drawStatus(fileInit, 'F', 2, RED);
}

// text and back are switched if active
void drawStatus(bool active, char letter, int dx, int text) {
  int x = AtomS3.Display.width() - dx * blinkSize;
  int y = AtomS3.Display.height() - blinkSize;
  AtomS3.Display.setTextFont(&fonts::TomThumb);
  AtomS3.Display.setTextSize(1);
  AtomS3.Display.fillRect(x, y, blinkSize , blinkSize , active ? text : BLACK);
  if (!active) {
    AtomS3.Display.drawRect(x, y, blinkSize , blinkSize , /*active ? BLACK :*/ text);
  }
  AtomS3.Display.setTextColor(active ? BLACK : text);
  AtomS3.Display.drawString(String(letter), x + centerText , y + centerText );
}

void updateScreen() {
  if (!oled) return;
  bool update = (displayUpdateDelay + lastDisplayUpdate) < millis();
  draw(update);
}
void draw() {
  if (!oled) return;
  draw(true);
}
void draw(bool update) {
  if (update) {
    AtomS3.Display.fillScreen(bg);
    drawGPSStatus();
    drawFileStatus();
    AtomS3.Display.setTextFont(&fonts::FreeSerif9pt7b);
    AtomS3.Display.setTextSize(1);
    AtomS3.Display.setTextColor(fg);
#define COLS 2
#define ROWS 8
    int col = 0;
    int row = 0;
    for (int i = 0; i < 15 ; i++) {
      //0 is ble
      int c = countNetworks[i];
      String ct = String(c);
      if (c > 1000) {
        c = c / 1000;
        ct = String (c) + "k";
      }
      AtomS3.Display.drawString((i < 10 ? "0" : "" ) + String(i) + " : " + (ct),
                                col * (AtomS3.Display.width() / COLS),
                                row * (AtomS3.Display.height() / ROWS));

      col++;
      if (col > (COLS - 1)) {
        row++;
        col = col % COLS;
      }
    }
    lastDisplayUpdate = millis();
  }
}
#endif

void loop() {
#ifdef S3OLED
  processButton();
#endif
#ifdef DomServer
  server.handleClient();
#endif
#ifdef S3OLED
  updateScreen();
#endif
  while (GPSSERIAL.available() > 0) {
    gps.encode(GPSSERIAL.read());
  }
#ifdef COMM_I2C
  for (uint8_t port = 0; port < NUM_PORTS; port++) {
    tcaselect(port);
    blinkLEDWhite();
    if (requestNetworkData(port)) {
#ifdef DomServer
      totalNetworksSent[port]++;
#endif
#ifdef S3OLED
      countNetworks[receivedNetworks[port].channel]++;
#endif
      logData(receivedNetworks[port], port);
    }
  }
#endif

#ifdef COMM_NOW
  //in case a sub rebooted
  long n = millis();
  if (n - lastBroadcastMillis > BROADCASTINTERVALL) {
    sendSubMsg();
  } else {
    //more possiblity to receive msgs
    delay(10);
  }
#endif
}

#ifdef DomServer
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>ESP32 Network Stats</title>";
  html += "<script>";
  html += "setInterval(function() { fetch('/data').then(response => response.text()).then(data => { document.getElementById('networkStats').innerHTML = data; }); }, 2000);";
  html += "</script></head><body>";
  html += "<h1>Network Statistics from Subs</h1>";
  html += "<div id='networkStats'></div>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  String data;
  data += "<h2>GPS Data:</h2>";
  data += "<p>Latitude: " + String(gps.location.lat(), 6) + "</p>";
  data += "<p>Longitude: " + String(gps.location.lng(), 6) + "</p>";
  data += "<p>HDOP: " + String(gps.hdop.value()) + "</p>";
  data += "<p>Satellites: " + String(gps.satellites.value()) + "</p>";

  for (int i = 0; i < NUM_PORTS; ++i) {

    data += "<h2>Port " + String(i) + ":</h2>";
    data += "<p>SSID: " + String(receivedNetworks[i].ssid) + "</p>";
    data += "<p>BSSID: " + String(receivedNetworks[i].bssid) + "</p>";
    data += "<p>RSSI: " + String(receivedNetworks[i].rssi) + "</p>";
    data += "<p>Security: " + String(receivedNetworks[i].security) + "</p>";
    data += "<p>Channel: " + String(receivedNetworks[i].channel) + "</p>";
    data += "<p>Total Networks Sent: " + String(totalNetworksSent[i]) + "</p>";
  }
  server.send(200, "text/plain", data);
}

#endif
#ifdef COMM_I2C
void tcaselect(uint8_t i) {
  if (i >= NUM_PORTS) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  Serial.println("[MASTER] Switched to I2C port: " + String(i));
}

bool requestNetworkData(uint8_t port) {
  Wire.requestFrom(i2c_slave_address, sizeof(NetworkInfo));
  blinkLEDWhite();
  if (Wire.available() < sizeof(NetworkInfo)) {
    return false;
  }
  Wire.readBytes((byte*)&receivedNetworks[port], sizeof(NetworkInfo));
  if (receivedNetworks[port].channel > 14) {
    //    Serial.println("[MASTER] Dropping network");
    return false;
  }
  return true;
}
#endif

void waitForGPSFix() {
  unsigned long lastBlink = 0;
  const unsigned long blinkInterval = 300;  // Time interval for LED blinking
  bool ledState = false;

  Serial.println("Waiting for GPS fix...");
#ifdef S3OLED
  drawGPSStatus();
#endif
  while (!gps.location.isValid()) {
    if (GPSSERIAL.available() > 0) {
      gps.encode(GPSSERIAL.read());
    }

#ifdef S3OLED
    processButton();
#endif
    // Non-blocking LED blink
    if (millis() - lastBlink >= blinkInterval) {
      lastBlink = millis();
      ledState = !ledState;
#ifdef S3OLED
      if (oled)
        AtomS3.Display.fillRect((AtomS3.Display.width() - blinkSize), (AtomS3.Display.height() - blinkSize) , blinkSize , blinkSize , ledState ? MAGENTA : BLACK);
#else
      led = led ? CRGB::Purple : CRGB::Black;
      FastLED.show();
#endif
    }
  }

#ifdef S3OLED
  drawGPSStatus();
#endif
  blinkLEDGreen();
  Serial.println("GPS fix obtained.");
}

void initializeFile() {
  int fileNumber = 0;
  bool isNewFile = false;

  // create a date stamp for the filename
  char fileDateStamp[16];
  sprintf(fileDateStamp, "%04d-%02d-%02d-",
          gps.date.year(), gps.date.month(), gps.date.day());

  do {
    fileName = "/wifi-scans-" + String(fileDateStamp) + String(fileNumber) + ".csv";
    isNewFile = !SD.exists(fileName);
    fileNumber++;
  } while (!isNewFile);

  if (isNewFile) {
    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("WigleWifi-1.4,appRelease=1.300000,model=GPS Kit,release=1.100000F+00,device=M5ATOMHydra,display=NONE,board=ESP32,brand=M5");
      dataFile.println("MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type");
      dataFile.close();
      Serial.println("New file created: " + fileName);
    }
  } else {
    Serial.println("Using existing file: " + fileName);
  }
#ifdef S3OLED
  fileInit = true;
  drawFileStatus();
#endif
}

void logData(const NetworkInfo& network, uint8_t port) {
#ifdef DomServer
  if (strcmp(ssid, network.ssid) == 0) {
    Serial.println("Skip");
    return;
  }
#endif
  if (gps.location.isValid()) {
    String utc = String(gps.date.year()) + "-" + gps.date.month() + "-" + gps.date.day() + " " + gps.time.hour() + ":" + gps.time.minute() + ":" + gps.time.second();
    String dataString = String(network.bssid) + "," + "\"" + network.ssid + "\"" + "," + network.security + "," + utc + "," + String(network.channel) + "," + String(network.rssi) + "," + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "," + String(gps.altitude.meters(), 2) + "," + String(gps.hdop.hdop(), 2);
    if (network.type == 'w' ) {
      dataString += ",WIFI";
    } else if (network.type == 'b' ) {
      dataString += ",BLE";
    }

    File dataFile = SD.open(fileName, FILE_APPEND);
    if (dataFile) {
#ifdef S3OLED
      fileInit = true;
#endif
      dataFile.println(dataString);
      dataFile.close();
      Serial.println("Data written: " + dataString);
      blinkLEDGreen();
    } else {
#ifdef S3OLED
      fileInit = false;
#endif
      Serial.println("Error opening " + fileName);
      blinkLEDRed();
    }
  } else {
    blinkLEDPurple();
  }
}


void blinkLEDWhite() {
#ifdef S3OLED
#else
  led = CRGB::White;
#endif
  blinkLED();
#ifdef S3OLED
  fg = WHITE;
#endif
}
void blinkLEDRed() {
#ifdef S3OLED
  fg = RED;
#else
  led = CRGB::Red;
#endif
  blinkLED();
#ifdef S3OLED
  fg = WHITE;
#endif
}
void blinkLEDGreen() {
#ifdef S3OLED
  fg = GREEN;
#else
  led = CRGB::Green;
#endif
  blinkLED();
#ifdef S3OLED
  fg = WHITE;
#endif
}
void blinkLEDPurple() {
#ifdef S3OLED
  fg = MAGENTA;
#else
  led = CRGB::Purple;
#endif
  blinkLED();
#ifdef S3OLED
  fg = WHITE;
#endif
}

void blinkLED() {
#ifdef S3OLED
  if (!oled)
    return;
  AtomS3.Display.fillRect((AtomS3.Display.width() - blinkSize), (AtomS3.Display.height() - blinkSize) , blinkSize , blinkSize , fg);
#else
  FastLED.show();
#endif
  delay(50);
#ifdef S3OLED
  AtomS3.Display.fillRect((AtomS3.Display.width() - blinkSize), (AtomS3.Display.height() - blinkSize) , blinkSize , blinkSize , bg);
#else
  led = CRGB::Black;
  FastLED.show();
#endif

}
