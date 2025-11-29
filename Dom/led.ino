

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
