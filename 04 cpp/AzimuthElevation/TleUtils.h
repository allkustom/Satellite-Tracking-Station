#include <fstream>
#include <vector>

#include "ArduinoJson.h"

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
