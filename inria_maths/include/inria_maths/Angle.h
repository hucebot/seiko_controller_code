#ifndef INRIA_MATHS_ANGLE_H
#define INRIA_MATHS_ANGLE_H

#include <cmath>

namespace inria {

/**
 * Angle degrees to radians and 
 * radians to degrees conversion
 */
constexpr double RadToDeg(double angle)
{
    return angle*180.0/M_PI;
}
constexpr double DegToRad(double angle)
{
    return angle*M_PI/180.0;
}

/**
 * Angle range normalization.
 *
 * @param angle in radian
 * @return given angle in radian 
 * bounded between -PI and PI
 */
inline double AngleBound(double angle)
{
    return 
        angle 
        - 2.0*M_PI*std::floor((angle + M_PI)/(2.0*M_PI));
}

/**
 * Compute angular distance
 * @param angleSrc angle source in radian
 * @param angleDst angle destination in radian
 * @return oriented distance from angleSrc to angleDst 
 * within -PI/2:PI/2 radian 
 */
inline double AngleDistance(double angleSrc, double angleDst) 
{    
    angleSrc = AngleBound(angleSrc);
    angleDst = AngleBound(angleDst);

    double max, min;
    if (angleSrc > angleDst) {
        max = angleSrc;
        min = angleDst;
    } else {
        max = angleDst;
        min = angleSrc;
    }

    double dist1 = max-min;
    double dist2 = 2.0*M_PI - max + min;
 
    if (dist1 < dist2) {
        if (angleSrc > angleDst) {
            return -dist1;
        } else {
            return dist1;
        }
    } else {
        if (angleSrc > angleDst) {
            return dist2;
        } else {
            return -dist2;
        }
    }
}

/**
 * Compute a weighted average 
 * between the two given angles in radian.
 *
 * @param weight1
 * @param angle1 in radian
 * @param weight2
 * @param angle2 in radian
 * @return averaged angle in -PI:PI.
 */
inline double AngleWeightedAverage(
    double weight1, double angle1, double weight2, double angle2)
{
    double x1 = std::cos(angle1);
    double y1 = std::sin(angle1);
    double x2 = std::cos(angle2);
    double y2 = std::sin(angle2);

    double meanX = weight1*x1 + weight2*x2;
    double meanY = weight1*y1 + weight2*y2;

    return std::atan2(meanY, meanX);
}

}

#endif

