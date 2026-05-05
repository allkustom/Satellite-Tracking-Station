wget https://github.com/astrolabe-www/artificial-calendar/raw/refs/heads/gh-pages/data/tles/all.json
wget -O ArduinoJson.h https://github.com/bblanchon/ArduinoJson/releases/download/v7.4.3/ArduinoJson-v7.4.3.h
git clone https://github.com/dnwrnr/sgp4.git

mmake() {
  g++ -I./sgp4/libsgp4 ./sgp4/libsgp4/*.cc test.cpp
}

clean_all() {
  rm -rf ./a.out all.json ArduinoJson.h sgp4
}
