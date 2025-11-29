void processButton() {
  //  Serial.println("Check Button");
#ifdef COREFIRE
  M5.update();

  if (M5.BtnA.wasReleased()) {
    oled = !oled;
    if (!oled) {
      Serial.println("Display Off");
      M5.Lcd.clear(BLACK);
    } else {
      Serial.println("Display On");
      draw(true);
    }
  }
#endif
#if defined(S3)
  AtomS3.update();

  if (AtomS3.BtnA.wasReleased()) {
    oled = !oled;
    if (!oled) {
      Serial.println("Display Off");
#ifdef S3OLED
      AtomS3.Display.clearDisplay();
    } else {
      Serial.println("Display On");
      draw(true);
    }
#endif
#ifdef MiniOLED
    display.clear();
    display.display();
    display.displayOff();
  } else {
    Serial.println("Display On");
    display.displayOn();
#endif
  }
#endif
}
