#pragma once

#include <cmath>

#include <CoordTopocentric.h>
#include <DateTime.h>
#include <Eci.h>
#include <Globals.h>
#include <Observer.h>
#include <SolarPosition.h>
#include <SGP4.h>
#include <Util.h>

#include "ArduinoJson.h"

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

namespace libsgp4 {

  class LunarPosition {
    public:
      LunarPosition() = default;
      Eci FindPosition(const DateTime& dt);
  };

  Eci LunarPosition::FindPosition(const DateTime& dt) {
      // Lunar formulas are standard in J2000, so convert to J2000 Julian Century (T)
      const double jd = dt.ToJulian();
      const double T = (jd - 2451545.0) / 36525.0;

      // 1. Fundamental Arguments (Mean Longitudes/Anomalies)
      const double L_prime = Util::Wrap360(218.316 + 481267.881 * T); // Moon's mean longitude
      const double D       = Util::Wrap360(297.850 + 445267.111 * T); // Mean elongation
      const double M       = Util::DegreesToRadians(Util::Wrap360(357.529 + 35999.050 * T)); // Sun's mean anomaly
      const double M_prime = Util::DegreesToRadians(Util::Wrap360(134.963 + 477198.868 * T)); // Moon's mean anomaly
      const double F       = Util::DegreesToRadians(Util::Wrap360(93.272 + 483202.018 * T)); // Moon's argument of latitude

      // 2. Ecliptic Longitude (lambda) - adding major perturbations
      // These specific terms account for the Equation of Center, Evection, and Variation
      double lambda = L_prime +
                      6.289 * sin(M_prime) +
                      1.274 * sin(2 * D - M_prime) +
                      0.658 * sin(2 * D) +
                      0.214 * sin(2 * M_prime) +
                      0.110 * sin(D);

      // 3. Ecliptic Latitude (beta)
      double beta = 5.128 * sin(F) +
                    0.280 * sin(M_prime + F) +
                    0.277 * sin(M_prime - F) +
                    0.173 * sin(2 * D - F) +
                    0.055 * sin(2 * D + F - M_prime) +
                    0.046 * sin(2 * D - F - M_prime) +
                    0.033 * sin(2 * D + F) +
                    0.017 * sin(2 * M_prime + F);

      // 4. Distance in Earth Radii (r) - Simplified
      double r = 385000.0 - 20905.0 * cos(M_prime) - 3699.0 * cos(2 * D - M_prime) - 2956.0 * cos(2 * D);

      // 5. Convert to Radians and ECI
      const double lambda_rad = Util::DegreesToRadians(lambda);
      const double beta_rad = Util::DegreesToRadians(beta);

      // Obliquity of the Ecliptic (epsilon)
      const double eps = Util::DegreesToRadians(23.43929 - 0.0130042 * T);

      // Spherical to Cartesian (Ecliptic Frame)
      double x_ecl = r * cos(beta_rad) * cos(lambda_rad);
      double y_ecl = r * cos(beta_rad) * sin(lambda_rad);
      double z_ecl = r * sin(beta_rad);

      // Rotate from Ecliptic to Equatorial (ECI)
      double x = x_ecl;
      double y = y_ecl * cos(eps) - z_ecl * sin(eps);
      double z = y_ecl * sin(eps) + z_ecl * cos(eps);

      return Eci(dt, Vector(x, y, z, r));
  }
} // namespace libsgp4

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
