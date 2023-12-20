#ifndef INRIA_MATHS_FILTEREXPONENTIAL_HPP
#define INRIA_MATHS_FILTEREXPONENTIAL_HPP

#include <cmath>
#include <inria_maths/Angle.h>

namespace inria {

/**
 * FilterExponential
 *
 * Type generic implementation
 * of the (varying time step) 
 * exponential filter.
 * val_{t+1} = coeff*val_{t} + (1.0-coeff)*input
 * The coefficient is computed dynamically
 * according to the sampling time step.
 *
 * Not thread safe but the functions update(), 
 * cutoffFrequency() and value() are RT safe 
 * (depending on the underlying template type).
 */
template <typename T>
class FilterExponential
{
    public:

        /**
         * Initialization with a default
         * time constant.
         *
         * @param cutoffFreq Optional default 
         * smoothing cutoff frequency in seconds.
         */
        FilterExponential(double cutoffFreq = 1000.0) :
            _state(T()),
            _isInitialized(false),
            _cutoffFreq(cutoffFreq)
        {
            reset();
        }

        /**
         * Update the internal filtered state with
         * in input value and the time step.
         *
         * @param value The new input value to be filtered.
         * @param dt The sampling time step in seconds.
         */
        void update(
            const T& value, 
            double dt)
        {
            //Compute the exponential smoothing coefficient
            //between 0.0 and 1.0.
            //0.0: No filter. Use purely raw input.
            //1.0: Full filter. Do not use input data.
            double coeff = getAlphaFromFreq(_cutoffFreq, dt);

            if (!_isInitialized) {
                //Assignment in initialization case
                _isInitialized = true;
                _state = value;
            } else {
                //Exponential update
                _state = computeAlphaFilter(_state, value, coeff);
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
         * @return read access to filtered value.
         */
        const T& value() const
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
         * given value.
         *
         * @param value Value to be assign
         * to current internal state.
         */
        void reset(const T& value)
        {
            _isInitialized = true;
            _state = value;
        }

        /**
         * Compute the exponential smoothing coefficient
         * between 0.0 and 1.0.
         * 0.0: No filter. Only update value is used.
         * 1.0: No update. Only state value is used.
         *
         * @param freq Cutoff frequency in Hz.
         * @param dt Time step in seconds.
         * @eturn the smoothing alpha coefficient between 0.0 and 1.0.
         */
        static double getAlphaFromFreq(double freq, double dt)
        {
            double omega = 2.0*M_PI*freq;
            double coeff = (1.0-omega*dt/2.0)/(1.0+omega*dt/2.0);
            
            //Clamp smoothing coefficient
            if (coeff < 0.0) {
                coeff = 0.0;
            }
            if (coeff > 1.0) {
                coeff = 1.0;
            }

            return coeff;
        }

        /**
         * Compute the exponential alpha filter from 
         * given smoothing coefficient.
         * 0.0: No filter. Only update value is used.
         * 1.0: No update. Only state value is used.
         *
         * @param valueState Original state value.
         * @param valueUpdate Update input value.
         * @param coeff Alpha coefficient between 0.0 and 1.0.
         * @return filtered value.
         */
        static T computeAlphaFilter(
            const T& valueState, 
            const T& valueUpdate, 
            double coeff)
        {
            return coeff*valueState + (1.0-coeff)*valueUpdate;
        }

    private:

        /**
         * Internal typed state.
         */
        T _state;

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

