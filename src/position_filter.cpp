#include "position_filter.h"

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

    // these are expensive
    float phi = atan2(distLat, distLon);
    float r = sqrt(distLat*distLat + distLon*distLon);

    //wait, we can just dot the location vector with the unit vector of the cable?

}
