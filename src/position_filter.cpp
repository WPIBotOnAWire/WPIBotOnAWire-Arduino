#include "position_filter.h"

#include <std_msgs/Float32.h>

const float RAD_PER_DMM = M_PI / (180 * 60 * 10000); //precision problem?
const float KAPPA = 0.25;

/**
 * Setup the geometry.
*/
Location::Location(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, float a)
{
    vertexH = a;

    LAT1DMM = lat1;
    LON1DMM = lon1;
    
    LAT2DMM = lat2;
    LON2DMM = lon2;

    //centerpoint; ignores great circle for such a short distance
    LAT0DMM = (LAT1DMM + LAT2DMM) / 2;
    LON0DMM = (LON1DMM + LON2DMM) / 2;

    metersPerDMMLat = 0.185185185; // 10,000 km over 90 degrees
    metersPerDMMLon = metersPerDMMLat * cos(LAT0DMM * RAD_PER_DMM);

    // find the delta
    float deltaXi = (LON2DMM - LON0DMM) * metersPerDMMLon;
    float deltaEta = (LAT2DMM - LAT0DMM) * metersPerDMMLat;

    // and convert to unit vector
    float xMax = sqrt(deltaXi*deltaXi + deltaEta*deltaEta);
    unitXi = deltaXi / xMax;
    unitEta = deltaEta / xMax;

    // sMax
    sMax = vertexH * sinh(xMax / vertexH);

    // default position to end by just copying over the pole location
    updateFromGPS(LAT2DMM, LON2DMM);
    sEstimated = sGPS;
}

std_msgs::Float32 loc_msg;
ros::Publisher pub_location("/location", &loc_msg);

void Location::InitFilter(ros::NodeHandle& nh)
{
    nh.advertise(pub_location);

    // default position to end by just copying over the pole location
    updateFromGPS(LAT2DMM, LON2DMM);
    sEstimated = sGPS;
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
    xGPS = distLat * unitXi + distLon * unitEta;
    sGPS = vertexH * sinh(xGPS / vertexH); // sinh is expensive, but the only one

    sEstimated = sEstimated + KAPPA * (sGPS - sEstimated);

    loc_msg.data = sGPS;
    pub_location.publish(&loc_msg);

    return sGPS;
}

float Location::updateFromEncoder(float deltaS)
{
    return sEstimated += deltaS;
}