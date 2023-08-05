#include "position_filter.h"

/**
 * Setup the geometry.
*/
Location::Location(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, float a)
{
    LAT1DMM = lat1;
    LON1DMM = lon1;
    
    LAT2DMM = lat2;
    LON2DMM = lon2;

    LAT0DMM = (LAT1DMM + LAT2DMM) / 2;
    LON0DMM = (LON1DMM + LON2DMM) / 2;

    metersPerDMMLat = 0; //?????
    metersPerDMMLon = metersPerDMMLat * cos(LAT0DMM / 0); //needs factor!!!

    // find the delta
    float deltaXi = (LON2DMM - LON0DMM) * metersPerDMMLon;
    float deltaEta = (LAT2DMM - LAT0DMM) * metersPerDMMLat;

    // and convert to unit vector
    float xMax = sqrt(deltaXi*deltaXi + deltaEta*deltaEta);
    unitXi = deltaXi / xMax;
    unitEta = deltaEta / xMax;
}

/**
 * Receives lat,lon in decimilliminutes and calculates x, the linear distance from the midpoint
 * between the two poles
*/
float Location::updateFromGPS(int32_t latDMM, int32_t lonDMM)
{
    int32_t deltaLat = latDMM - LAT0DMM;
    int32_t deltaLon = lonDMM - LON0DMM;

    float distLat = deltaLat * metersPerDMMLat;
    float distLon = deltaLon * metersPerDMMLon;

    // dot the GPS vector with the unit vector of the cable to get x 
    // (ie, project the GPS reading to conform to the cable)
    float currX = distLat * unitXi + distLon * unitEta;
    float currS = vertexH * sinh(currX / vertexH);

    // do I want/need to update x,s datums?

    return currS;
}
