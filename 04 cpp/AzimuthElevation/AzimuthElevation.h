#include <ctime>
#include <fstream>
#include <string>
#include "json.hpp"

using json = nlohmann::json;

struct DateTime {
  unsigned int year;
  unsigned int month;
  unsigned int day;
  unsigned int hour;
  unsigned int minute;
  unsigned int second;
};

struct Location {
  float lat;
  float lon;
  int tz;
};

struct AzEl {
  float azimuth;
  float elevation;
};

void read_tles(const std::string& filename, json& data) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Could not open file." << std::endl;
    return;
  }

  file >> data;
  return;
}

time_t toUnixTime(const DateTime& localDateTime, const Location& location) {
  struct tm timeinfo = {0};
  timeinfo.tm_year = localDateTime.year - 1900;
  timeinfo.tm_mon = localDateTime.month - 1;
  timeinfo.tm_mday = localDateTime.day;
  timeinfo.tm_hour = localDateTime.hour;
  timeinfo.tm_min = localDateTime.minute;
  timeinfo.tm_sec = localDateTime.second;

  return timegm(&timeinfo) - (location.tz * 60 * 60);
}

AzEl solarAzimuthElevation(const DateTime& localDateTime, const Location& location) {
  return AzEl{0, 0};
}

AzEl lunarAzimuthElevation(const DateTime& localDateTime, const Location& location) {
  return AzEl{0, 0};
}

AzEl satelliteAzimuthElevation(const DateTime& localDateTime, const Location& location, json& tle) {
  return AzEl{0, 0};
}
