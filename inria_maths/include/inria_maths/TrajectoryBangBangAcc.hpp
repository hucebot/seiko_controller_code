#ifndef INRIA_MATHS_TRAJECTORYBANGBANGACC_HPP
#define INRIA_MATHS_TRAJECTORYBANGBANGACC_HPP

#include <Eigen/Dense>
#include <inria_maths/Polynomial.hpp>
#include <inria_maths/AxisAngle.h>
#include <inria_maths/Clamp.h>

namespace inria {

/**
 * TrajectoryBangBangAcc
 *
 * Minimal time trajectory generator with 
 * continuous velocity, discontinuous acceleration, 
 * bound on velocity discreet bang-bang acceleration
 */
class TrajectoryBangBangAcc
{
    public:

        /**
         * Initialization and trajectory generation.
         *
         * @param posInit Start position.
         * @param velInit Start velocity.
         * @param posEnd Final position.
         * @param velEnd Final velocity.
         * @param maxVel Absolute velocity limit.
         * @param maxAcc Absolute acceleration limit.
         */
        TrajectoryBangBangAcc(
            double posInit, double velInit,
            double posEnd, double velEnd,
            double maxVel, double maxAcc);

        /**
         * Evaluation of the trajectory position 
         * and its first, second and third derivatives.
         *
         * @param t The evaluation time.
         * @return trajectory evaluation at t.
         */
        double pos(double t) const;
        double vel(double t) const;
        double acc(double t) const;

        /**
         * @return minimum and maximum time 
         * abscissa value for which the 
         * trajectory is defined
         */
        double min() const;
        double max() const;

    private:

        /**
         * Total defined trajectory time length
         */
        double _timeLength;

        /**
         * Position begin and end value
         * for outbound extrapolation
         */
        double _posInit;
        double _posEnd;

        /**
         * If true, long range three phase mode
         * is used. If false, short range cubic 
         * polynomial mode is used.
         */
        bool _isPhaseMode;

        /**
         * State variable for long range
         * bang-bang trajectory mode
         */
        double _velInit;
        double _velMiddle;
        double _velEnd;
        double _accBegin;
        double _accEnd;
        double _timeAccBegin;
        double _timeMiddle;
        double _timeAccEnd;

        /**
         * State variable for short range
         * cubic polynomial trajectory mode
         */
        double _timePoly1;
        double _timePoly2;
        Polynomial _poly1;
        Polynomial _poly2;
};

/**
 * FilterBangBangAcc
 *
 * Implement filter continuously applying the bang-bang 
 * acceleration planning.
 * Bound input signal velocity and acceleration.
 * 
 * @param T Eigen::Vector or double type.
 */
template <typename T>
class FilterBangBangAcc
{
    public:

        /**
         * Initialization with default parameters
         */
        FilterBangBangAcc(
            double maxVel = 1e3, double maxAcc = 1e6) :
            _pos(),
            _vel(),
            _isInitialized(false),
            _maxVel(maxVel),
            _maxAcc(maxAcc)
        {
        }
        
        /**
         * Update the internal filtered state with
         * in input value and the time step.
         *
         * @param input The new input value to be filtered.
         * @param dt The sampling time step in seconds.
         */
        void update(
            const T& input, 
            double dt)
        {
            //Initialization if needed
            if (!_isInitialized) {
                reset(input);
            }

            //Apply bang-bang trajectory filter for each component
            for (int i=0;i<input.size();i++) {
                //Clamp current velocity in case the 
                //maximum velocity is changed dynamically
                _vel(i) = ClampAbsolute(_vel(i), _maxVel);
                inria::TrajectoryBangBangAcc traj(
                    _pos(i), _vel(i), 
                    input(i), 0.0,
                    _maxVel, _maxAcc);
                _pos(i) = traj.pos(dt);
                _vel(i) = traj.vel(dt);
            }
        }

        /**
         * @return read access to filtered value
         */
        const T& value() const
        {
            return _pos;
        }
        const T& valuePos() const
        {
            return _pos;
        }
        const T& valueVel() const
        {
            return _vel;
        }
        
        /**
         * @return access to maximum velocity 
         * and acceleration parameter
         */
        const double& maxVel() const
        {
            return _maxVel;
        }
        double& maxVel()
        {
            return _maxVel;
        }
        const double& maxAcc() const
        {
            return _maxAcc;
        }
        double& maxAcc()
        {
            return _maxAcc;
        }

        /**
         * Set the internal state as 
         * uninitialized. Its value will be 
         * assigned at next call of update().
         */
        void reset()
        {
            _isInitialized = false;
        }

        /**
         * Reset the internal state to
         * given value.
         *
         * @param value Value to be assign
         * to current internal state.
         */
        void reset(const T& value)
        {
            _isInitialized = true;
            _pos = value;
            _vel.setZero();
        }

    private:

        /**
         * Position, velocity and last 
         * computed acceleration state
         */
        T _pos;
        T _vel;
        
        /**
         * If false, the current state is 
         * not yet initialized and should be
         * set at next update() call.
         */
        bool _isInitialized;

        /**
         * Parameters
         */
        double _maxVel;
        double _maxAcc;
};

/**
 * Specialization for double type
 */
template <>
inline void FilterBangBangAcc<double>::reset(const double& value)
{
    _isInitialized = true;
    _pos = value;
    _vel = 0.0;
}
template <>
inline void FilterBangBangAcc<double>::update(const double& input, double dt)
{
    //Initialization if needed
    if (!_isInitialized) {
        reset(input);
    }

    //Clamp current velocity in case the 
    //maximum velocity is changed dynamically
    _vel = ClampAbsolute(_vel, _maxVel);
    //Apply bang-bang trajectory filter for each component
    inria::TrajectoryBangBangAcc traj(
        _pos, _vel, 
        input, 0.0,
        _maxVel, _maxAcc);
    _pos = traj.pos(dt);
    _vel = traj.vel(dt);
}

/**
 * FilterBangBangAccRotation
 *
 * Implement FilterBangBangAcc for rotation
 */
class FilterBangBangAccRotation
{
    public:

        /**
         * Initialization with default parameters
         */
        FilterBangBangAccRotation(
            double maxVel = 1e3, double maxAcc = 1e6) :
            _posMat(),
            _velAxis(),
            _isInitialized(false),
            _maxVel(maxVel),
            _maxAcc(maxAcc)
        {
        }
        
        /**
         * Update the internal filtered state with
         * in input value and the time step.
         *
         * @param input The new input value to be filtered.
         * @param dt The sampling time step in seconds.
         */
        void update(
            const Eigen::Matrix3d& input, 
            double dt)
        {
            //Initialization if needed
            if (!_isInitialized) {
                reset(input);
            }
            
            //Apply bang-bang trajectory filter for each component in the Lie algebra
            Eigen::Vector3d inputVect = MatrixToAxis(input*_posMat.transpose());
            Eigen::Vector3d nextVect = Eigen::Vector3d::Zero();
            for (int i=0;i<3;i++) {
                //Clamp current velocity in case the 
                //maximum velocity is changed dynamically
                _velAxis(i) = ClampAbsolute(_velAxis(i), _maxVel);
                inria::TrajectoryBangBangAcc traj(
                    0.0, _velAxis(i), 
                    inputVect(i), 0.0,
                    _maxVel, _maxAcc);
                nextVect(i) = traj.pos(dt);
                _velAxis(i) = traj.vel(dt);
            }
            _posMat = AxisToMatrix(nextVect)*_posMat;
        }

        /**
         * @return read access to filtered value
         */
        const Eigen::Matrix3d& value() const
        {
            return _posMat;
        }
        const Eigen::Matrix3d& valuePos() const
        {
            return _posMat;
        }
        const Eigen::Vector3d& valueVel() const
        {
            return _velAxis;
        }
        
        /**
         * @return access to maximum velocity 
         * and acceleration parameter
         */
        const double& maxVel() const
        {
            return _maxVel;
        }
        double& maxVel()
        {
            return _maxVel;
        }
        const double& maxAcc() const
        {
            return _maxAcc;
        }
        double& maxAcc()
        {
            return _maxAcc;
        }

        /**
         * Set the internal state as 
         * uninitialized. Its value will be 
         * assigned at next call of update().
         */
        void reset()
        {
            _isInitialized = false;
        }

        /**
         * Reset the internal state to
         * given value.
         *
         * @param value Value to be assign
         * to current internal state.
         */
        void reset(const Eigen::Matrix3d& value)
        {
            _isInitialized = true;
            _posMat = value;
            _velAxis.setZero();
        }

    private:

        /**
         * Position, velocity and last 
         * computed acceleration state
         */
        Eigen::Matrix3d _posMat;
        Eigen::Vector3d _velAxis;
        
        /**
         * If false, the current state is 
         * not yet initialized and should be
         * set at next update() call.
         */
        bool _isInitialized;

        /**
         * Parameters
         */
        double _maxVel;
        double _maxAcc;
};

}

#endif

