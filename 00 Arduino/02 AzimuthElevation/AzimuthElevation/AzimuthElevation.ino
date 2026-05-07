#include <iostream>
#include <vector>

#define ARDUINOJSON_ENABLE_STD_STRING 1
#define ARDUINOJSON_ENABLE_STD_STREAM 1
#include "ArduinoJson.h"

#include "AzimuthElevation.h"

std::vector<DateTime> TESTS{
  DateTime{ .year = 1972, .month = 3, .day = 31, .hour = 22, .minute = 22, .second = 22 },
  DateTime{ .year = 1981, .month = 10, .day = 10, .hour = 10, .minute = 10, .second = 10 },
  DateTime{ .year = 2000, .month = 10, .day = 10, .hour = 0, .minute = 30, .second = 0 },
  DateTime{ .year = 2010, .month = 1, .day = 2, .hour = 3, .minute = 4, .second = 55 },
  DateTime{ .year = 2020, .month = 8, .day = 17, .hour = 9, .minute = 3, .second = 5 },
  DateTime{ .year = 2026, .month = 10, .day = 10, .hour = 21, .minute = 11, .second = 44 },
};

Location NYC{ .lat = 40.731266, .lon = -73.997060, .tz = -4 };
Location FOR{ .lat = -3.731862, .lon = -38.526669, .tz = -3 };

std::vector<std::string> SATS { "gps", "glonass", "sci", "goes" };

JsonDocument allTles;
JsonObject gps0tle;

const char* gps0string = "{\"gps\":[{\"name\":\"GPS BIIR-5  (PRN 22)\", \"tle\":[\"1 26407U 00040A   26123.91544338  .00000012  00000+0  00000+0 0  9995\", \"2 26407  54.8595 216.9205 0121523 302.3502  55.8691  2.00557395189054\"]}]}";

void setup() {
  Serial.begin(9600);
  deserializeJson(allTles, gps0string);
  gps0tle = allTles["gps"][0].as<JsonObject>();

  while (!Serial && millis() < 5000) {
    delay(100);
  }

  Serial.print("\nSUN\n");
  for (int idx = 0; idx < TESTS.size(); idx++) {
    AzEl sunAzEl = solarAzimuthElevation(TESTS[idx], NYC);
    Serial.print(sunAzEl.azimuth);
    Serial.print(" ");
    Serial.println(sunAzEl.elevation);
  }

  Serial.print("\nMOON\n");
  for (int idx = 0; idx < TESTS.size(); idx++) {
    AzEl moonAzEl = lunarAzimuthElevation(TESTS[idx], NYC);
    Serial.print(moonAzEl.azimuth);
    Serial.print(" ");
    Serial.println(moonAzEl.elevation);
  }

  Serial.print("\nGPS[0]\n");
  for (int idx = 0; idx < TESTS.size(); idx++) {
    AzEl gps0AzEl = satelliteAzimuthElevation(TESTS[idx], NYC, gps0tle);
    Serial.print(gps0AzEl.azimuth);
    Serial.print(" ");
    Serial.println(gps0AzEl.elevation);
  }
}

void loop() {
  delay(5000);
}
