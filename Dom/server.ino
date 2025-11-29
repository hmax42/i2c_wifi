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
