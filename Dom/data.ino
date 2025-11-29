
#ifdef COMM_NOW
// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  int node = myData.boardId;
  Serial.println("Received " + String(myData.ssid));
#ifdef DomServer
  totalNetworksSent[node]++;
#endif
#if defined(S3OLED) || defined(MiniOLED) || defined(COREFIRE)
  countNetworks[myData.channel]++;
#endif
  logData(myData, node);
}

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void  sendSubMsg() {
  esp_now_send(broadcastAddress, (uint8_t*)&dummy, sizeof(dummy));
  lastBroadcastMillis = millis();
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

/*
  void p(char X) {
  if (X < 16) {Serial.print("0");}
   Serial.println(X, HEX);
  }
*/


int s3Slaves[] = {4};
int number_of_s3Slaves = sizeof(s3Slaves) / sizeof(int);

bool requestNetworkData(uint8_t port) {
  blinkLEDWhite();
  Wire.requestFrom(i2c_slave_address, sizeof(NetworkInfo));
  int r = Wire.available();
  //Serial.println("received " + String(r) + " bytes");
  if ( r < sizeof(NetworkInfo)) {
    //Serial.println("nothing");
    return false;
  }

  //FIXME slave type unknown
  //dummy byte https://github.com/espressif/arduino-esp32/issues/10145
  //  if (number_of_s3Slaves > 0) {
  //    for (int s = 0; s < number_of_s3Slaves; s++) {
  //      if (s3Slaves[s] == port) {
  //        byte b = 0x00;
  //        Wire.readBytes((byte*)&b, 1); //dummy
  //        Serial.println("[MASTER] Read dummy");
  //      }
  //    }
  //  }

  Wire.readBytes((byte*)&receivedNetworks[port], sizeof(NetworkInfo));
  //FIXME
  /*
    if (receivedNetworks[port].bssid[1] == 0x3a || receivedNetworks[port].bssid[2] == 0x3a ) {
    //colon at 1 or 2
    String str = String(receivedNetworks[port].bssid) + "," + "\"" + receivedNetworks[port].ssid + "\"" + "," + receivedNetworks[port].security + "," + String(receivedNetworks[port].channel) + "," + String(receivedNetworks[port].rssi);
    printString(str);
    }
  */
  if (port == 0) {
    String str = String(receivedNetworks[port].bssid) + "," + "\"" + receivedNetworks[port].ssid + "\"" + "," + receivedNetworks[port].security + "," + String(receivedNetworks[port].channel) + "," + String(receivedNetworks[port].rssi);
    printString(str);
    //printBytes((byte*)&receivedNetworks[port], sizeof(NetworkInfo));
  }
  /*
    if (((byte*)&receivedNetworks[port])[32+1] == 0x3a || ((byte*)&receivedNetworks[port])[32+2] == 0x3a ) {
    //colon at 1 or 2
    printBytes((byte*)&receivedNetworks[port], sizeof(NetworkInfo));
    }
  */
  /*
     20:21:35.442 -> Network received:
     20:21:35.442 -> C:F4:51:BB:88:1D,"LAN-341166",WPA2_PSK],119,1543503871
     20:21:35.442 -> 43 3A 46 34 3A 35 31 3A 42 42 3A 38 38 3A 31 44 2C 22 4C 41 4E 2D 33 34 31 31 36 36 22 2C 57 50 41 32 5F 50 53 4B 5D 2C 31 31 39 2C 31 35 34 33 35 30 33 38 37 31
     missing lead byte, probably one tail byte to much!
     missing lead E of MAC adress, leading W of ssid, leading [ of security...
  */


  if (receivedNetworks[port].channel > 14) {
    //Serial.println("[MASTER] Dropping network");
    return false;
  }
  return true;
}

void printString(String str) {
  Serial.println("Network received: ");
  Serial.println(str);
  for (int i = 0; i < str.length(); i++) {
    Serial.print(str.charAt(i), HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void printBytes(byte* str, int len) {
  Serial.println("Network received: ");
  for (int i = 0; i < len; i++) {
    Serial.print(str[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
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

#if defined(S3OLED) || defined(MiniOLED) || defined(COREFIRE)
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
      led = ledState ? CRGB::Purple : CRGB::Black;
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
    fileName = String("/") + String(FILEPREFIX) + String("wifi-scans-") + String(fileDateStamp) + String(fileNumber) + String(".csv");
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

void logData(const NetworkInfo & network, uint8_t port) {
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
