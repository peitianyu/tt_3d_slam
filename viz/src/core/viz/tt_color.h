#ifndef __COLOR_H__
#define __COLOR_H__

#include <iostream>

namespace viz{

struct Color
{
    uint8_t r;
    uint8_t g;
    uint8_t b;

    Color(uint8_t r, uint8_t g, uint8_t b):r(r), g(g), b(b)
    {}

    Color():Color(0, 0, 0)
    {}
};

#ifdef USE_PCL_VIZ
#define COLOR_RED Color(255, 0, 0)
#define COLOR_GREEN Color(0, 255, 0)
#define COLOR_BLUE Color(0, 0, 255)
#define COLOR_YELLOW Color(255, 255, 0)
#define COLOR_PURPLE Color(255, 0, 255)
#define COLOR_CYAN Color(0, 255, 255)
#define COLOR_WHITE Color(255, 255, 255)
#define COLOR_BLACK Color(0, 0, 0)
#define COLOR_GRAY Color(128, 128, 128)
#else
#define COLOR_RED Color(0, 0, 255)
#define COLOR_GREEN Color(0, 255, 0)
#define COLOR_BLUE Color(255, 0, 0)
#define COLOR_YELLOW Color(0, 255, 255)
#define COLOR_PURPLE Color(255, 0, 255)
#define COLOR_CYAN Color(255, 255, 0)
#define COLOR_WHITE Color(255, 255, 255)
#define COLOR_BLACK Color(0, 0, 0)
#define COLOR_GRAY Color(128, 128, 128)
#endif // USE_PCL_VIZ

} // namespace viz

#endif // __COLOR_H__