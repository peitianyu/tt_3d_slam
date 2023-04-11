#ifndef __POINT_H__
#define __POINT_H__

#include <iostream>

struct Point
{
    float x;
    float y;
    float z;
    float intensity;

    Point(float x = 0.0, float y = 0.0, float z = 0.0, float intensity = 0.0) : 
        x(x), y(y), z(z), intensity(intensity) {}

    friend std::ostream& operator<<(std::ostream& os, const Point& point)
    {
        os << point.x << " " << point.y << " " << point.z << " " << point.intensity;
        return os;
    }
};


#endif // __POINT_H__