async function getTleFromUrl(url) {
  const tleRes = await fetch(url);
  const tleTxt = await tleRes.text();
  const tleLines = tleTxt.split("\n");

  const tles = [];

  for (let ln = 0; ln < tleLines.length - 2; ln += 3) {
    tles.push({
      name: tleLines[ln + 0].trim(),
      tle: [
        tleLines[ln + 1].trim(),
        tleLines[ln + 2].trim(),
      ]
    });
  }
  return tles;
}

async function fetchTles(root="") {
  const satGroups = ["goes", "noaa", "starlink", "kuiper", "sss", "sci", "gps", "glonass", "galileo", "beidou"];

  // { k: f"../data/etc/{k}.txt" for k in satGroups }
  const URLS = satGroups.reduce((acc, k) => ({ ...acc, [k]: `${root}data/tles/${k}.txt` }), {});

  const ps = satGroups.map(k => getTleFromUrl(URLS[k]));
  const data = await Promise.all(ps);

  return satGroups.reduce((acc, k, idx) => ({ ...acc, [k]: data[idx] }), {});
}

export { getTleFromUrl, fetchTles };
