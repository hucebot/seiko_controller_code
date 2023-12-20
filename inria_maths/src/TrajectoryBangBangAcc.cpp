#include <cmath>
#include <stdexcept>
#include <inria_maths/TrajectoryBangBangAcc.hpp>

namespace inria {

TrajectoryBangBangAcc::TrajectoryBangBangAcc(
    double posInit, double velInit,
    double posEnd, double velEnd,
    double maxVel, double maxAcc) :
    _timeLength(0.0),
    _posInit(posInit),
    _posEnd(posEnd),
    _isPhaseMode(false),
    _velInit(velInit),
    _velMiddle(0.0),
    _velEnd(velEnd),
    _accBegin(0.0),
    _accEnd(0.0),
    _timeAccBegin(0.0),
    _timeMiddle(0.0),
    _timeAccEnd(0.0),
    _timePoly1(0.0),
    _timePoly2(0.0),
    _poly1(),
    _poly2()
{
    //Check limits validity
    if (maxVel <= 0.0 || maxAcc <= 0.0) {
        throw std::logic_error(
            "inria::TrajectoryBangBangAcc: "
            "Invalid limits.");
    }
    //Forbid out of bound starting 
    //or final velocity
    double epsilon = 1e-3;
    if (
        std::fabs(velInit) > maxVel+epsilon || 
        std::fabs(velEnd) > maxVel+epsilon
    ) {
        throw std::logic_error(
            "inria::TrajectoryBangBangAcc: "
            "Out of limit velocity.");
    }
    
    //Assign saturated velocity and acceleration/deceleration
    //limit depending on motion direction
    if (posEnd >= posInit) {
        _velMiddle = maxVel;
        _accBegin = maxAcc;
        _accEnd = -maxAcc;
    } else {
        _velMiddle = -maxVel;
        _accBegin = -maxAcc;
        _accEnd = maxAcc;
    }
    //Compute acceleration and deceleration 
    //time to/from maximum velocity
    _timeAccBegin = (_velMiddle-velInit)/_accBegin;
    _timeAccEnd = (velEnd-_velMiddle)/_accEnd;
    //Numerical error safety
    if (_timeAccBegin < 0.0) {
        _timeAccBegin = 0.0;
    }
    if (_timeAccEnd < 0.0) {
        _timeAccEnd = 0.0;
    }

    //Compute the distance travelled during 
    //acceleration and deceleration time
    double distAccBegin = std::fabs(
        velInit*_timeAccBegin + 
        0.5*_accBegin*_timeAccBegin*_timeAccBegin);
    double distAccEnd = std::fabs(
        _velMiddle*_timeAccEnd + 
        0.5*_accEnd*_timeAccEnd*_timeAccEnd);
    //Total distance to travel
    double distNow = std::fabs(posEnd - posInit);
    
    if (distNow >= (distAccBegin + distAccEnd)) {
        //If the distance to travel is larger and the 
        //acceleration and deceleration distance, a 3 phases 
        //trajectory with saturated velocity in the middle
        //is used
        _isPhaseMode = true;
        //Distance travelled at maximum velocity
        double distMiddle = distNow - (distAccBegin+distAccEnd);
        //Time spend at maximum velocity
        _timeMiddle = std::fabs(distMiddle/_velMiddle);
        //Compute total trajectory time
        _timeLength = _timeAccBegin + _timeMiddle + _timeAccEnd;
    } else {
        //Else the distance is too short to reach the maximum
        //velocity. A two phase acceleration/deceleration phase
        //trajectory is optimized for (second order polynomial
        //spline with velocity continuity and maximum acceleration).
        _isPhaseMode = false;
        //Solve for time length of each of the two phases
        //(second order equation) to reach desired position and
        //velocity using positive then negative maximum acceleration
        _timePoly1 = 0.0;
        _timePoly2 = 0.0;
        double acc = (posEnd >= posInit) ? maxAcc : -maxAcc;
        double deltaPositive = 
            velInit*velInit + 
            velEnd*velEnd + 
            2.0*acc*(posEnd-posInit);
        if (deltaPositive < 0.0) {
            throw std::logic_error(
                "inria::TrajectoryBangBangAcc: "
                "Negative discriminant.");
        }
        double rootTime1 = 
            (std::sqrt(2.0)*std::sqrt(deltaPositive)-2.0*velInit)/(2.0*acc);
        double rootTime2 = 
            (-std::sqrt(2.0)*std::sqrt(deltaPositive)-2.0*velInit)/(2.0*acc);
        //Select the positive time length solution for first phase
        _timePoly1 = std::max(rootTime1, rootTime2);
        //Compute the second phase time length
        _timePoly2 = _timePoly1 - (velEnd-velInit)/acc;
        //If the target vel is too hight and the target position is too close,
        //it can not be reached directly. The other solution is to go away from
        //the target position and then go toward it again.
        //Another two phases constant acceleration is computed (same maths)
        //but with the acceleration inversed (first decelerate, then accelerate).
        if (_timePoly1 < 0.0 || _timePoly2 < 0.0) {
            //Compute the same solution with acc = -acc
            double deltaNegative = 
                velInit*velInit + 
                velEnd*velEnd + 
                2.0*(-acc)*(posEnd-posInit);
            if (deltaNegative >= 0.0) {
                double rootTime3 = 
                    (std::sqrt(2.0)*std::sqrt(deltaNegative)-2.0*velInit)/(-2.0*acc);
                double rootTime4 = 
                    (-std::sqrt(2.0)*std::sqrt(deltaNegative)-2.0*velInit)/(-2.0*acc);
                //Select the positive time length solution for first phase
                double time1 = std::max(rootTime3, rootTime4);
                //Compute the second phase time length
                double time2 = time1 - (velEnd-velInit)/(-acc);
                //If this solution is feasible, then we use it
                if (time1 >= 0.0 && time2 >= 0.0) {
                    _timePoly1 = time1;
                    _timePoly2 = time2;
                    acc = -acc;
                }
            }
        }
        //Clamp the phase time to zero if previous case has not found a solution
        //(the target state is then not reached, but this should not happen...)
        _timePoly1 = std::max(_timePoly1, 0.0);
        _timePoly2 = std::max(_timePoly2, 0.0);
        //Assign total trajectory length
        _timeLength = _timePoly1 + _timePoly2;
        //Assign the two part polynomials
        _poly1 = inria::Polynomial(2);
        _poly1(0) = posInit;
        _poly1(1) = velInit;
        _poly1(2) = 0.5*acc;
        _poly2 = inria::Polynomial(2);
        _poly2(0) = _poly1.pos(_timePoly1);
        _poly2(1) = _poly1.vel(_timePoly1);
        _poly2(2) = -0.5*acc;
    }
}

double TrajectoryBangBangAcc::pos(double t) const
{
    //Outbound extrapolation
    if (t < 0.0) {
        return _posInit - _velInit*t;
    }
    if (t > _timeLength) {
        return _posEnd + (t-_timeLength)*_velEnd;
    }

    if (_isPhaseMode) {
        if (t <= _timeAccBegin) {
            return _posInit + _velInit*t + 0.5*_accBegin*t*t;
        } else if (t <= _timeAccBegin+_timeMiddle) {
            double tt = t - _timeAccBegin;
            return _posInit + _velInit*_timeAccBegin 
                + 0.5*_accBegin*_timeAccBegin*_timeAccBegin 
                + _velMiddle*tt;
        } else {
            double tt = t - _timeAccBegin - _timeMiddle;
            return _posInit + _velInit*_timeAccBegin 
                + 0.5*_accBegin*_timeAccBegin*_timeAccBegin 
                + _velMiddle*_timeMiddle
                + _velMiddle*tt + 0.5*_accEnd*tt*tt;
        }
    } else {
        if (t <= _timePoly1) {
            return _poly1.pos(t);
        } else {
            return _poly2.pos(t-_timePoly1);
        }
    }
}
double TrajectoryBangBangAcc::vel(double t) const
{
    //Outbound extrapolation
    if (t < 0.0) {
        return _velInit;
    }
    if (t > _timeLength) {
        return _velEnd;
    }

    if (_isPhaseMode) {
        if (t <= _timeAccBegin) {
            return _velInit + _accBegin*t;
        } else if (t <= _timeAccBegin+_timeMiddle) {
            return _velMiddle;
        } else {
            double tt = t - _timeAccBegin - _timeMiddle;
            return _velMiddle + _accEnd*tt;
        }
    } else {
        if (t <= _timePoly1) {
            return _poly1.vel(t);
        } else {
            return _poly2.vel(t-_timePoly1);
        }
    }
}
double TrajectoryBangBangAcc::acc(double t) const
{
    //Outbound extrapolation
    if (t < 0.0 || t > _timeLength) {
        return 0.0;
    }

    if (_isPhaseMode) {
        if (t <= _timeAccBegin) {
            return _accBegin;
        } else if (t <= _timeAccBegin+_timeMiddle) {
            return 0.0;
        } else {
            return _accEnd;
        }
    } else {
        if (t <= _timePoly1) {
            return _poly1.acc(t);
        } else {
            return _poly2.acc(t-_timePoly1);
        }
    }
}

double TrajectoryBangBangAcc::min() const
{
    return 0.0;
}
double TrajectoryBangBangAcc::max() const
{
    return _timeLength;
}

}

