

#ifdef S3OLED
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
#endif


#if defined(S3OLED) || defined(MiniOLED) || defined(COREFIRE)
long displayUpdateDelay = DISPLAYTIME;
long lastDisplayUpdate = - DISPLAYTIME;

void draw(bool update) {
  if (update) {
#ifdef S3OLED
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
      AtomS3.Display.drawString((i == 0 ? String("bt") : ((i < 10 ? "0" : "" ) + String(i))) + " : " + (ct),
                                col * (AtomS3.Display.width() / COLS),
                                row * (AtomS3.Display.height() / ROWS));

      col++;
      if (col > (COLS - 1)) {
        row++;
        col = col % COLS;
      }
    }
#elif defined(MiniOLED)
    //MiniOLED
    int c = countNetworks[currentChannelOled];
    String ct = String(c);
    if (c > 1000) {
      c = c / 1000;
      ct = String (c) + "k";
    }
    //Serial.println(String(currentChannelOled) + " : " + ct);
    display.setFont(ArialMT_Plain_16);
    display.clear();
    display.drawRect(x + 0, y + 0, 72, 40);
    int i = currentChannelOled;
    display.drawString(x + 0 + 5, y + 10 + 2, (i == 0 ? String("bt") : ((i < 10 ? "0" : "" ) + String(i))) + ":" + ct);
    display.display();
    currentChannelOled++;
    if (currentChannelOled == 15) {
      currentChannelOled = 0;
    }
#elif defined(COREFIRE)
    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(bg);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(fg);
#define COLS 3
#define ROWS 5
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
      M5.Lcd.drawString((i == 0 ? String("bt") : ((i < 10 ? "0" : "" ) + String(i))) + " : " + (ct),
                        5 + col * (M5.Lcd.width() / COLS),
                        5 + row * (M5.Lcd.height() / ROWS));

      col++;
      if (col > (COLS - 1)) {
        row++;
        col = col % COLS;
      }
    }
#endif
    lastDisplayUpdate = millis();
  }
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



#endif
