#include <iostream>
#include <vector>

#include "json.hpp"
#include "AzimuthElevation.h"

using json = nlohmann::json;

std::vector<DateTime> TESTS {
  DateTime{ .year = 1972, .month = 3,  .day = 31, .hour = 22, .minute = 22, .second = 22 },
  DateTime{ .year = 1981, .month = 10, .day = 10, .hour = 10, .minute = 10, .second = 10 },
  DateTime{ .year = 2000, .month = 10, .day = 10, .hour = 0,  .minute = 30, .second = 0  },
  DateTime{ .year = 2010, .month = 1,  .day = 2,  .hour = 3,  .minute = 4,  .second = 55 },
  DateTime{ .year = 2020, .month = 8,  .day = 17, .hour = 9,  .minute = 3,  .second = 5  },
  DateTime{ .year = 2026, .month = 10, .day = 10, .hour = 21, .minute = 11, .second = 44 },
};

Location NYC { .lat = 40.731266, .lon = -73.997060, .tz = -4 };
Location FOR { .lat = -3.731862, .lon = -38.526669, .tz = -3 };

int main() {
  json allTles, gps0tle;
  read_tle_file(std::string{"./all.json"}, allTles);
  gps0tle = allTles.at("gps").at(0);
  std::cout << gps0tle.dump(2) << "\n";

  std::cout << "\nSUN\n";
  for (int idx = 0; idx < TESTS.size(); idx++) {
    AzEl sunAzEl = solarAzimuthElevation(TESTS[idx], NYC);
    std::cout << sunAzEl.azimuth << " " << sunAzEl.elevation << "\n";
  }

  std::cout << "\nMOON\n";
  for (int idx = 0; idx < TESTS.size(); idx++) {
    AzEl sunAzEl = lunarAzimuthElevation(TESTS[idx], NYC);
    std::cout << sunAzEl.azimuth << " " << sunAzEl.elevation << "\n";
  }

  std::cout << "\nGPS[0]\n";
  for (int idx = 0; idx < TESTS.size(); idx++) {
    AzEl gps0AzEl = satelliteAzimuthElevation(TESTS[idx], NYC, gps0tle);
    std::cout << gps0AzEl.azimuth << " " << gps0AzEl.elevation << "\n";
  }

  return 0;
}
