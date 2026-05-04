## libsgp4

https://github.com/dnwrnr/sgp4/blob/master/sattrack/sattrack.cc

## SuperNOVAS
https://sigmyne.github.io/SuperNOVAS/doc/html/group__apparent.html

[`novas_approx_sky_pos()`](https://sigmyne.github.io/SuperNOVAS/doc/html/group__apparent.html#ga4cb01609f76b0ff8487817b9446a3a62)

[`NOVAS_MOON`](https://sigmyne.github.io/SuperNOVAS/doc/html/cpp/novas_8h.html#a219df36b21dc4476656e708d14d08045a9fcf58133828600a062725ced448cfcf)

```cpp
novas_frame frame = ...;   // observer location and time of observation
sky_pos apparent = {};     // apparent data structure to populate...

// Calculate apparent position, say in true-of-date (TOD) system
if(novas_approx_sky_pos(NOVAS_MOON, &frame, NOVAS_TOD, &apparent) != 0) {
  // Oops something went wrong...
  return -1;
}
```
