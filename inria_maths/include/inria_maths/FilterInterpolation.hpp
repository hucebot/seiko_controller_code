#ifndef INRIA_MATHS_FILTERINTERPOLATION_HPP
#define INRIA_MATHS_FILTERINTERPOLATION_HPP

#include <cmath>

namespace inria {

/**
 * FilterInterpolation
 *
 * Dynamic linear interpolation as filter
 */
template <typename T>
class FilterInterpolation
{
    public:

        /**
         * Initialization.
         */
        FilterInterpolation() :
            _timeState(-1.0),
            _timeLength(-1.0),
            _valueStart(),
            _valueEnd()
        {
        }

        /**
         * Reset interpolation with starting current state.
         *
         * @param valueStart Initial value for interpolation.
         * @param timeLength Interpolation time length.
         */
        void reset(const T& valueStart, double timeLength = -1.0)
        {
            _timeState = timeLength;
            _timeLength = timeLength;
            _valueStart = valueStart;
            _valueEnd = valueStart;
        }

        /**
         * Update interpolation state.
         *
         * @param valueEnd Final value to interpolate toward.
         * @param dt Time step.
         */
        void update(const T& valueEnd, double dt)
        {
            //Reset if not initialized
            if (_timeState <= 0.0 || _timeLength <= 0.0) {
                reset(valueEnd);
            }

            _timeState -= dt;
            _valueEnd = valueEnd;
        }

        /**
         * @return read access to filtered interpolated value
         */
        T value() const
        {
            if (_timeState <= 0.0 || _timeLength <= 0.0) {
                return _valueEnd;
            } else {
                double alpha = _timeState/_timeLength;
                if (alpha > 1.0) {
                    alpha = 1.0;
                }
                if (alpha < 0.0) {
                    alpha = 0.0;
                }
                return alpha*_valueStart + (1.0-alpha)*_valueEnd;
            }
        }

    private:
        
        /**
         * Remaining time until 
         * interpolation end
         */
        double _timeState;

        /**
         * Interpolation time length 
         * from start to end
         */
        double _timeLength;

        /**
         * Generic initial and final value 
         * for linear interpolation
         */
        T _valueStart;
        T _valueEnd;
};

}

#endif

