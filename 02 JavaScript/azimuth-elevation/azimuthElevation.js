import * as satellite from "https://cdn.jsdelivr.net/npm/satellite.js@6.0.2/dist/satellite.es.js";

const deg2rad = (d) => d * Math.PI / 180.0;
const rad2deg = (r) => r * 180.0 / Math.PI;

const sinDeg = (d) => Math.sin(deg2rad(d));
const cosDeg = (d) => Math.cos(deg2rad(d));

// ----------------------------
// Solar azimuth/elevation (NOAA-style)
// Returns azimuth_deg (0=N,90=E,180=S,270=W) and elevation_deg
// https://www.sciencedirect.com/science/article/pii/S0960148121004031
function solarAzimuthElevation(localDate, localTime, location) {
  const { year, month, day } = localDate;
  const { hour, minute, second } = localTime;
  const { lat:latDeg, lon:lonDeg, tz:tzOffsetHours } = location;

  const utcHour = hour - tzOffsetHours;
  const utcInHour = utcHour + minute / 60.0 + second / 3600.0;
  const lat_r = deg2rad(latDeg);
  const lon_r = deg2rad(lonDeg);

  const dtUtc = new Date(Date.UTC(year, month - 1, day, utcHour, minute, second));
  const jd = satellite.jday(dtUtc);
  const days = jd - 2451545.0;

  // Mean longitude of the Sun
  const L = (280.460 + 0.9856474 * days) % 360.0;

  // Sun's right ascension and declination
  const { rtasc, decl } = satellite.sunPos(jd);
  const alpha = rad2deg(rtasc);
  const dec_r = decl;

  // Equation of Time, apparent solar time minus mean solar time
  const Emin = (L - alpha) * 4.0;

  // Longitude of the subsolar point
  const lambda_s = -15.0 * (utcInHour - 12.0 + Emin / 60.0);
  const lambda_s_r = deg2rad(lambda_s);

  // Vector from observer to center of sun
  const Sx = Math.cos(dec_r) * Math.sin(lambda_s_r - lon_r);
  const Sy = Math.cos(lat_r) *  Math.sin(dec_r) - Math.sin(lat_r) * Math.cos(dec_r) * Math.cos(lambda_s_r - lon_r);
  const Sz = Math.sin(lat_r) * Math.sin(dec_r) + Math.cos(lat_r) * Math.cos(dec_r) * Math.cos(lambda_s_r - lon_r);

  const azimuth_r = Math.atan2(Sx, Sy);
  const zenith_r = Math.acos(Sz);

  const azimuth = (rad2deg(azimuth_r) + 360) % 360;
  const zenith = rad2deg(zenith_r);
  const elevation = 90.0 - zenith;

  return { azimuth, elevation };
}


function lunarAzimuthElevation(localDate, localTime, location) {
  const rad = Math.PI / 180;

  const { year, month, day } = localDate;
  const { hour, minute, second } = localTime;
  const { lat:latDeg, lon:lonDeg, tz:tzOffsetHours } = location;

  const utcHour = hour - tzOffsetHours;

  const dtUtc = new Date(Date.UTC(year, month - 1, day, utcHour, minute, second));
  const jd = satellite.jday(dtUtc);
  const days = jd - 2451545.0;

  // mean longitude
  const L = (218.316 + 13.176396 * days) * rad;

  // mean anomaly
  const M = (134.963 + 13.064993 * days) * rad;

  // mean distance
  const F = (93.272 + 13.229350 * days) * rad;

  // mean elongation
  const D = (297.8501921 + 12.19074912 * days) * rad;

  // ecliptic longitude
  const lonEcl =
    L
    + (6.289 * rad) * Math.sin(M)
    + (1.274 * rad)  * Math.sin(2*D - M)
    + (0.658 * rad)  * Math.sin(2*D)
    + (0.214 * rad)  * Math.sin(2*M)
    + (0.110 * rad)  * Math.sin(D);

  // ecliptic latitude
  const latEcl =
    (5.128 * rad) * Math.sin(F)
    + (0.280 * rad) * Math.sin(M + F)
    + (0.277 * rad) * Math.sin(M - F)
    + (0.173 * rad) * Math.sin(2*D - F)
    + (0.055 * rad) * Math.sin(2*D + F - M)
    + (0.046 * rad) * Math.sin(2*D - F - M)
    + (0.033 * rad) * Math.sin(2*D + F)
    + (0.017 * rad) * Math.sin(2*M + F);

  // obliquity
  const e = (23.439291 - 0.0000004 * days) * rad; 

  // right ascension
  const ra = Math.atan2(
    Math.sin(lonEcl) * Math.cos(e) - Math.tan(latEcl) * Math.sin(e),
    Math.cos(lonEcl)
  );

  // declination
  const dec = Math.asin(
    Math.sin(latEcl) * Math.cos(e) +
    Math.cos(latEcl) * Math.sin(e) * Math.sin(lonEcl)
  );

  const lw = -lonDeg * rad;
  const phi = latDeg * rad;

  // sidereal time
  const H = ((280.16 + 360.9856235 * days) * rad - lw) - ra;

  const alt = Math.asin(
    Math.sin(phi) * Math.sin(dec) +
    Math.cos(phi) * Math.cos(dec) * Math.cos(H)
  );

  const az = Math.atan2(
    Math.sin(H),
    Math.cos(H) * Math.sin(phi) - Math.tan(dec) * Math.cos(phi)
  );

  return {
    azimuth: (az / rad + 180.0 + 360) % 360,
    elevation: alt / rad
  };
}


function satelliteAzimuthElevation(localDate, localTime, location, tle) {
  const { year, month, day } = localDate;
  const { hour, minute, second } = localTime;
  const { lat:latDeg, lon:lonDeg, tz:tzOffsetHours } = location;
  const [tleLine1, tleLine2] = tle;

  // observer
  const observerGd = {
    latitude: satellite.degreesToRadians(latDeg),
    longitude: satellite.degreesToRadians(lonDeg),
    height: 0
  };

  // local date time
  const utcHour = hour - tzOffsetHours;
  const dtUtc = new Date(Date.UTC(year, month - 1, day, utcHour, minute, second));

  // GMST for some of the coordinate transforms
  const gmst = satellite.gstime(dtUtc);

  // Initialize a satellite record
  const satrec = satellite.twoline2satrec(tleLine1, tleLine2);

  // Propagate satellite using js date
  const positionAndVelocity = satellite.propagate(satrec, dtUtc);

  if (!positionAndVelocity) {
    return {
      azimuth: 0,
      elevation: 0,
    };
  }

  // The positionAndVelocity result is a pair of ECI coordinates.
  const positionEci = positionAndVelocity.position;

  // ECF and  Look Angles
  const positionEcf = satellite.eciToEcf(positionEci, gmst);
  const lookAngles = satellite.ecfToLookAngles(observerGd, positionEcf);

  return {
    azimuth: rad2deg(lookAngles.azimuth),
    elevation: rad2deg(lookAngles.elevation),
  };
}

export { solarAzimuthElevation, lunarAzimuthElevation, satelliteAzimuthElevation };
