import requests

def get_tles_from_url(url):
  tleRes = requests.get(url)
  tleTxt = tleRes.text
  tleLines = tleTxt.split("\n")

  tles = []

  for idx in range(0, len(tleLines) - 2, 3):
    tles.append({
      "name": tleLines[idx + 0].strip(),
      "tle": [
        tleLines[idx + 1].strip(),
        tleLines[idx + 2].strip(),
      ]
    })

  return tles

def fetch_tles(root=""):
  satGroups = ["goes", "noaa", "starlink", "kuiper", "sss", "sci", "gps", "glonass", "galileo", "beidou"]

  urls = { k: f"{root}data/tles/{k}.txt" for k in satGroups }
  data = { k: get_tles_from_url(u) for k,u in urls.items() }

  return data
