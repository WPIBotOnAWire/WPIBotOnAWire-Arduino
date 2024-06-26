#pragma once
#include <ros.h>
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
 * 
 * Lat/Lon are stored as "decimilliminutes" (DMM) so they can be stored as integers.
 * Divide by 10000 to get minutes, where there are 60 minutes in a degree.
*/

/**
 * TODO: Add more statistics:
 * - innovation
 * - deviation of GPS from the cable
 * - bias
 * 
 * TODO: Take time lags into account
 * 
 * TODO: Do I _really_ want this here? Or should I put it on the Jetson?
*/
class Location
{
private:
    //float xFused = 0; // estimated position along the wire, measured from the middle
    float sEstimated = 0; // estimated position between the poles, measure from the middle

    float xGPS = 0; // observed position from GPS
    float sGPS = 0; // observed position from GPS

    float xMax, sMax;

    int32_t LAT1DMM, LON1DMM; // lat, lon of pole 1 (negative end); stored as DMM
    int32_t LAT2DMM, LON2DMM; // lat, lon of pole 2 (positive end); stored as DMM
    int32_t LAT0DMM, LON0DMM; // lat, lon of midpoint between poles; stored as DMM

    float metersPerDMMLat;
    float metersPerDMMLon;

    // unit vector of the cable, where xi is east/west; eta is north/south
    float unitXi;
    float unitEta;

    float vertexH; // a geometric parameter of the catenary

public:
    Location(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, float a);
    void InitFilter(ros::NodeHandle& nh);
    float updateFromEncoder(float deltaS);
    float updateFromGPS(int32_t latDMM, int32_t lonDMM);
};