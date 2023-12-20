#ifndef INRIA_MATHS_FILTERVELCOMMAND_HPP
#define INRIA_MATHS_FILTERVELCOMMAND_HPP

#include <inria_maths/FilterExponential.hpp>
#include <inria_maths/TrajectoryBangBangAcc.hpp>

namespace inria {

/**
 * FilterVelCommand
 *
 * Filter velocity command integration into
 * a filtered position command that is clamped not to be
 * too far from a state position.
 */
template <typename T = double>
class FilterVelCommand
{
    public:

        /**
         * Default initialization
         */
        FilterVelCommand() :
            _isInitialized(false),
            _clampRadius(),
            _target(),
            _filterLowpass(),
            _filterBangbang()
        {
        }

        /**
         * Assign filter parameters
         *
         * @param clampRadius Distance for clamping signal from state.
         * @param cutoffFreq Cutoff frequency of lowpass filter.
         * @param maxVel Maximum signal velocity.
         * @param maxAcc Maximum signal acceleration.
         */
        void setParameters(
            double clampRadius,
            double cutoffFreq,
            double maxVel,
            double maxAcc)
        {
            _clampRadius = clampRadius;
            _filterLowpass.cutoffFrequency() = cutoffFreq;
            _filterBangbang.maxVel() = maxVel;
            _filterBangbang.maxAcc() = maxAcc;
        }

        /**
         * Reset internal filter to uninitialized
         */
        void reset()
        {
            _isInitialized = false;
        }

        /**
         * Update internal state with given 
         * velocity command.
         *
         * @param dt Time step.
         * @param vel Velocity command to be integrated.
         * @param state Current command state from which 
         * the target signal can not be farer than the 
         * clamping radius.
         */
        void update(
            double dt,
            const T& vel, const T& state)
        {
            //Initialization
            if (!_isInitialized) {
                _isInitialized = true;
                _target = state;
                _filterLowpass.reset(_target);
                _filterBangbang.reset(_target);
            }

            //Velocity integration
            _target += dt*vel;
            //Clamping to state sphere
            if (_target > state + _clampRadius) {
                _target = state + _clampRadius;
            }
            if (_target < state - _clampRadius) {
                _target = state - _clampRadius;
            }
            //Update filters
            _filterLowpass.update(_target, dt);
            _filterBangbang.update(_filterLowpass.value(), dt);
        }

        /**
         * @return internal pre-filtering target position
         */
        const T& target() const
        {
            return _target;
        }

        /**
         * @return filtered position command signal
         */
        const T& value() const
        {
            return _filterBangbang.value();
        }

    private:

        /**
         * If false, the filter will be initialized 
         * at next update() call
         */
        bool _isInitialized;

        /**
         * Radius for command clamping
         */
        double _clampRadius;

        /**
         * Target position before filtering
         */
        T _target;

        /**
         * Lowpass and max velocity and acceleration filters
         */
        FilterExponential<T> _filterLowpass;
        FilterBangBangAcc<T> _filterBangbang;
};

}

#endif

