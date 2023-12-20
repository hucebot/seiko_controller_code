#ifndef INRIA_MATHS_FILTEREXPONENTIALROTATION_HPP
#define INRIA_MATHS_FILTEREXPONENTIALROTATION_HPP

#include <Eigen/Dense>

namespace inria {

/**
 * FilterExponentialRotation
 *
 * Implementation of the exponential 
 * filter for 3d rotation matrix and quaternion
 * using quaternion interpolation.
 * val_{t+1} = coeff*val_{t} + (1.0-coeff)*input
 * The coefficient is computed dynamically
 * according to the sampling time step.
 *
 * Not thread safe but the functions update(), 
 * cutoffFrequency() and valueMatrix(), valueQuaternion() 
 * are RT safe.
 */
class FilterExponentialRotation
{
    public:
        
        /**
         * Initialization with a default
         * time constant.
         *
         * @param cutoffFreq Optional default 
         * smoothing cutoff frequency in seconds.
         */
        FilterExponentialRotation(double cutoffFreq = 1000.0) :
            _state(Eigen::Quaterniond::Identity()),
            _isInitialized(false),
            _cutoffFreq(cutoffFreq)
        {
            reset();
        }

        /**
         * Update the internal filtered state with
         * in input value and the time step.
         *
         * @param value The new rotation matrix 
         * or quaternion be filtered.
         * @param dt The sampling time step in seconds.
         */
        void update(
            const Eigen::Matrix3d& value, 
            double dt)
        {
            update(Eigen::Quaterniond(value), dt);
        }
        void update(
            const Eigen::Quaterniond& value, 
            double dt)
        {
            //Compute the exponential smoothing coefficient
            //between 0.0 and 1.0.
            //0.0: No filter. Use purely raw input.
            //1.0: Full filter. Do not use input data.
            double omega = 2.0*M_PI*_cutoffFreq;
            double coeff = (1.0-omega*dt/2.0)/(1.0+omega*dt/2.0);

            //Clamp smoothing coefficient
            if (coeff < 0.0) {
                coeff = 0.0;
            }
            if (coeff > 1.0) {
                coeff = 1.0;
            }

            if (!_isInitialized) {
                reset(value);
            } else {
                _state = 
                    _state.slerp(1.0-coeff, value);
            }
        }

        /**
         * @return read or write and read direct 
         * access to the lowpass cutoff frequency.
         */
        const double& cutoffFrequency() const
        {
            return _cutoffFreq;
        }
        double& cutoffFrequency()
        {
            return _cutoffFreq;
        }

        /**
         * @return read access to filtered value
         * converted back as rotation matrix or quaternion
         */
        Eigen::Matrix3d valueMatrix() const
        {
            return _state.toRotationMatrix();
        }
        Eigen::Quaterniond valueQuaternion() const
        {
            return _state;
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
         * given rotation matrix or quaternion.
         *
         * @param value Orientation to be assign
         * to current internal state.
         */
        void reset(const Eigen::Matrix3d& mat)
        {
            _isInitialized = true;
            _state = Eigen::Quaterniond(mat);
        }
        void reset(const Eigen::Quaterniond& quat)
        {
            _isInitialized = true;
            _state = quat;
        }

    private:
        
        /**
         * Internal state.
         */
        Eigen::Quaterniond _state;

        /**
         * If false, the current state is 
         * not yet initialized and should be
         * set at next update() call.
         */
        bool _isInitialized;

        /**
         * Cutoff frequency of the lowpass filter
         */
        double _cutoffFreq;
};

}

#endif

