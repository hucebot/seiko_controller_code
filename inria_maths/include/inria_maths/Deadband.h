#ifndef INRIA_MATHS_DEADBAND_H
#define INRIA_MATHS_DEADBAND_H

namespace inria {

/**
 * Deadband filter around zero.
 *
 * @param value Value to be filtered.
 * @param width Half range width (centered around zero)
 * on which the input value is zeroed.
 * @param isContinuous If false, the neutral zone is simply applied
 * and a discontinuity at +/- width value is present. 
 * If true, an offset is applied on value to have smooth linear slope.
 *
 * @return the filtered value.
 */
inline double Deadband(
    double value, double width, bool isContinuous)
{
    if (width < 0.0) {
        width = 0.0;
    }

    if (isContinuous) {
        if (value >= width) {
            return value - width;
        } else if (value <= -width) {
            return value + width;
        } else {
            return 0.0;
        }
    } else {
        if (std::fabs(value) <= width) {
            return 0.0;
        } else {
            return value;
        }
    }
}

/**
 * Apply Deadband filter on given
 * vector norm.
 */
template <typename T>
inline T DeadbandVectorNorm(
    const T& vect, double width, bool isContinuous)
{
    if (width < 0.0) {
        width = 0.0;
    }

    double norm = vect.norm();
    if (isContinuous) {
        if (norm >= width) {
            return vect.normalized()*(norm - width);
        } else {
            return 0.0*vect;
        }
    } else {
        if (norm <= width) {
            return 0.0*vect;
        } else {
            return vect;
        }
    }
}

/**
 * Apply Deadband filter on all 
 * components of given vector.
 */
template <typename T>
inline T DeadbandVectorComponent(
    const T& vect, double width, bool isContinuous)
{
    T vectOut = vect;
    for (size_t i=0;i<vect.size();i++) {
        vectOut[i] = Deadband(vect[i], width, isContinuous);
    }
    return vectOut;
}

}

#endif

