from astropy import units as ast_u
from astropy.coordinates import GCRS, TEME, CartesianDifferential, CartesianRepresentation
from astropy.coordinates import get_body, AltAz, EarthLocation
from astropy.time import Time as AstroTime

from sgp4.api import Satrec, SGP4_ERRORS

from datetime import datetime


def pad(x):
  if len(str(x)) < 2:
    return f"000{x}"[-2:]
  else:
    return str(x)


def to_astro_location(location):
  latDeg, lonDeg = location["lat"], location["lon"]
  return EarthLocation(lat=latDeg*ast_u.deg, lon=lonDeg*ast_u.deg)


def to_astro_time(localDate, localTime, location):
  year, month, day = localDate["year"], localDate["month"], localDate["day"]
  hour, minute, second = localTime["hour"], localTime["minute"], localTime["second"]
  tzOffsetHours = location["tz"]

  date_str = f"{year}-{pad(month)}-{pad(day)}"
  time_str = f"{pad(hour)}:{pad(minute)}:{pad(second)}"
  datetime_str = f"{date_str}T{time_str}"
  dt = datetime.fromisoformat(datetime_str)
  dt_tz = dt.timestamp() - (tzOffsetHours * 60 * 60)

  return AstroTime(dt_tz, format="unix", scale="utc")


# Transform GCRS to Local Azimuth / Elevation
def gcrs_to_azel(gcrs, obstime, location):
  altaz = AltAz(obstime=obstime, location=location)
  local_altaz = gcrs.transform_to(altaz)

  return {
    "azimuth": local_altaz.az.deg,
    "elevation": local_altaz.alt.deg,
  }


def solar_azimuth_elevation(localDate, localTime, location):
  astro_dt = to_astro_time(localDate, localTime, location)
  astro_loc = to_astro_location(location)

  # Sun position in GCRS
  sun_gcrs = get_body(body="sun", time=astro_dt)
  return gcrs_to_azel(sun_gcrs, astro_dt, astro_loc)


def lunar_azimuth_elevation(localDate, localTime, location):
  astro_dt = to_astro_time(localDate, localTime, location)
  astro_loc = to_astro_location(location)

  # Moon position GCRS
  moon_gcrs = get_body(body="moon", time=astro_dt)
  return gcrs_to_azel(moon_gcrs, astro_dt, astro_loc)


# https://docs.astropy.org/en/latest/coordinates/satellites.html
def satellite_azimuth_elevation(localDate, localTime, location, tle):
  astro_dt = to_astro_time(localDate, localTime, location)
  astro_loc = to_astro_location(location)
  tleLine1, tleLine2 = tle

  satellite = Satrec.twoline2rv(tleLine1, tleLine2)

  error_code, teme_p, teme_v = satellite.sgp4(astro_dt.jd1, astro_dt.jd2)
  if error_code == 1:
    return { "azimuth": 0, "elevation": 0 }
  elif error_code != 0:
    print(error_code, SGP4_ERRORS[error_code])

  teme_p = CartesianRepresentation(teme_p*ast_u.km)
  teme_v = CartesianDifferential(teme_v*ast_u.km/ast_u.s)
  sat_teme = TEME(teme_p.with_differentials(teme_v), obstime=astro_dt)

  # TEME -> GCRS
  sat_gcrs = sat_teme.transform_to(GCRS(obstime=astro_dt))
  return gcrs_to_azel(sat_gcrs, astro_dt, astro_loc)
