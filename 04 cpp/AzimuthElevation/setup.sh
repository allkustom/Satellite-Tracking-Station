wget https://github.com/nlohmann/json/raw/refs/heads/develop/single_include/nlohmann/json.hpp
wget https://github.com/astrolabe-www/artificial-calendar/raw/refs/heads/gh-pages/data/tles/all.json
git clone https://github.com/dnwrnr/sgp4.git

# git clone https://github.com/Hopperpop/Sgp4-Library.git
# pushd Sgp4-Library/src
# sed -E -i '/^.+Arduino.+/s/^/\/\/ /g' sgp4pred.cpp
# sed -E -i 's/\bunix\b/unix_/g' sgp4pred.h sgp4pred.cpp
# sed -E -i 's/\bdaylight\b/Daylight/g' sgp4pred.h sgp4pred.cpp
# sed -E -i 's/\beclipsed\b/Eclipsed/g' sgp4pred.h sgp4pred.cpp
# sed -E -i 's/\blighted\b/Lighted/g' sgp4pred.h sgp4pred.cpp
# popd

mmake() {
  g++ -I./sgp4/libsgp4 ./sgp4/libsgp4/*.cc test.cpp
}

clean_all() {
  rm -rf ./a.out all.json json.hpp sgp4
}
