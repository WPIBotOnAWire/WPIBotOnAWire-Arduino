#pragma once

#include <Arduino.h>

/**
 * We assume that the ends of the cable are at the same height to make the maths
 * a little easier. Can be changed later.
 * 
 * We measure s, the distance along the cable, from the center, which hangs 'sag' meters
 * below the poles.
 * 
 * We measure x, the linear distance between poles, from the center of the cable.
 * 
 * That puts the ends at +/- maxX, maxS.
 * 
 * The encoders measure changes in s.
 * The GPS measures changes in x (by way of lat/lon).
*/
class Location
{
private:
    float s = 0; // distance along the wire, measured from the middle
    float x = 0; // distance between the poles, measure from the middle

    float xMax, sMax;
    uint16_t LAT1, LON1; // lat, lon of pole 1 (negative end)
    uint16_t LAT2, LON2; // lat, lon of pole 2 (positive end)

public:
    Location(void) {}
    float updateFromEncoder(float deltaS);
    float updateFromGPS(uint16_t lat, uint16_t lon);
};