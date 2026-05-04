#include <ctime>
#include <fstream>
#include "json.hpp"

#include <CoordTopocentric.h>
#include <Observer.h>
#include <SolarPosition.h>
#include <SGP4.h>

using json = nlohmann::json;

constexpr double PI = 3.14159265358979323846;

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

void read_tle_file(const std::string& filename, json& data) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Could not open file." << std::endl;
    return;
  }

  file >> data;
  return;
}

time_t toUnixTime(const DateTime& localDateTime) {
  struct tm timeinfo = {0};
  timeinfo.tm_year = localDateTime.year - 1900;
  timeinfo.tm_mon = localDateTime.month - 1;
  timeinfo.tm_mday = localDateTime.day;
  timeinfo.tm_hour = localDateTime.hour;
  timeinfo.tm_min = localDateTime.minute;
  timeinfo.tm_sec = localDateTime.second;

  return timegm(&timeinfo);
}

time_t toUnixTime(const DateTime& localDateTime, const Location& location) {
  time_t noTz = toUnixTime(localDateTime);
  return noTz - (location.tz * 60 * 60);
}

inline float toDeg(double rad) {
  return rad / PI * 180;
}

AzEl solarAzimuthElevation(const DateTime& localDateTime, const Location& location) {
  libsgp4::Observer obs(location.lat, location.lon, 0);

  libsgp4::SolarPosition sun = libsgp4::SolarPosition();

  libsgp4::DateTime dt = libsgp4::DateTime(
    localDateTime.year, localDateTime.month, localDateTime.day,
    localDateTime.hour, localDateTime.minute, localDateTime.second).AddHours(-location.tz);

  libsgp4::Eci eci = sun.FindPosition(dt);
  libsgp4::CoordTopocentric topo = obs.GetLookAngle(eci);

  return AzEl{ .azimuth = toDeg(topo.azimuth), .elevation = toDeg(topo.elevation) };
}

AzEl lunarAzimuthElevation(const DateTime& localDateTime, const Location& location) {
  return AzEl{0, 0};
}

AzEl satelliteAzimuthElevation(const DateTime& localDateTime, const Location& location, json& tle) {
  std::string tle_name = tle.at("name").get<std::string>();
  std::string tle_line1 = tle.at("tle").at(0).get<std::string>();
  std::string tle_line2 = tle.at("tle").at(1).get<std::string>();

  libsgp4::Observer obs(location.lat, location.lon, 0);

  libsgp4::Tle sat = libsgp4::Tle(tle_name, tle_line1, tle_line2);
  libsgp4::SGP4 sgp4(sat);

  libsgp4::DateTime dt = libsgp4::DateTime(
    localDateTime.year, localDateTime.month, localDateTime.day,
    localDateTime.hour, localDateTime.minute, localDateTime.second).AddHours(-location.tz);

  libsgp4::Eci eci = sgp4.FindPosition(dt);
  libsgp4::CoordTopocentric topo = obs.GetLookAngle(eci);

  return AzEl{ .azimuth = toDeg(topo.azimuth), .elevation = toDeg(topo.elevation) };
}
