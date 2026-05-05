#pragma once

#include "DateTime.h"
#include "Eci.h"

#include "Globals.h"
#include "Util.h"

#include <cmath>

namespace libsgp4
{

class LunarPosition
{
  public:
    LunarPosition() = default;
    Eci FindPosition(const DateTime& dt);
};

Eci LunarPosition::FindPosition(const DateTime& dt)
{
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
                    0.214 * sin(2 * M_prime) -
                    0.186 * sin(M);

    // 3. Ecliptic Latitude (beta)
    double beta = 5.128 * sin(F) +
                  0.280 * sin(M_prime + F) +
                  0.278 * sin(M_prime - F) +
                  0.212 * sin(2 * D - F);

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
