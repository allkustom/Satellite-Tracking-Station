#include <fstream>
#include <vector>

#include "ArduinoJson.h"

#include <CoordTopocentric.h>
#include <Observer.h>
#include <SolarPosition.h>
#include <SGP4.h>
#include <Util.h>

#include "LunarPosition.h"

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
  double azimuth;
  double elevation;
};

void read_tle_file(const std::string& filename, std::vector<std::string>& sats, JsonDocument& data) {
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Could not open file." << std::endl;
    return;
  }

  if (sats.size()) {
    JsonDocument filter;
    for (int idx = 0; idx < sats.size(); idx++) {
      filter[sats[idx]] = true;
    }
    deserializeJson(data, file, DeserializationOption::Filter(filter));
  } else {
    deserializeJson(data, file);
  }

  return;
}

void read_tle_file(const std::string& filename, JsonDocument& data) {
  std::vector<std::string> sats{ };
  read_tle_file(filename, sats, data);
  return;
}

template <typename T>
AzEl getAzimuthElevation(const DateTime& localDateTime, const Location& location, T& astroObject) {
  libsgp4::Observer obs(location.lat, location.lon, 0);

  libsgp4::DateTime dt = libsgp4::DateTime(
    localDateTime.year, localDateTime.month, localDateTime.day,
    localDateTime.hour, localDateTime.minute, localDateTime.second).AddHours(-location.tz);

  libsgp4::Eci eci = astroObject.FindPosition(dt);
  libsgp4::CoordTopocentric topo = obs.GetLookAngle(eci);

  return AzEl {
    libsgp4::Util::RadiansToDegrees(topo.azimuth),
    libsgp4::Util::RadiansToDegrees(topo.elevation)
  };
}

AzEl solarAzimuthElevation(const DateTime& localDateTime, const Location& location) {
  libsgp4::SolarPosition sun = libsgp4::SolarPosition();
  return getAzimuthElevation(localDateTime, location, sun);
}

AzEl lunarAzimuthElevation(const DateTime& localDateTime, const Location& location) {
  libsgp4::LunarPosition moon = libsgp4::LunarPosition();
  return getAzimuthElevation(localDateTime, location, moon);
}

AzEl satelliteAzimuthElevation(const DateTime& localDateTime, const Location& location, const JsonDocument& tle) {
  libsgp4::Tle sat = libsgp4::Tle(tle["name"], tle["tle"][0], tle["tle"][1]);
  libsgp4::SGP4 sgp4(sat);
  return getAzimuthElevation(localDateTime, location, sgp4);
}
