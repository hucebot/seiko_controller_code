#ifndef INRIA_MATHS_FILTERDIFFERENTIATOR2_HPP
#define INRIA_MATHS_FILTERDIFFERENTIATOR2_HPP

#include <cmath>
#include <vector>
#include <stdexcept>
#include <inria_maths/FilterExponential.hpp>

namespace inria {

/**
 * FilterDifferentiator2
 *
 * Lowpass filtered finite difference differentiator
 */
template <typename T>
class FilterDifferentiator2
{
    public:

        /**
         * Initialization.
         *
         * @param historySize Number of past data points 
         * to store in the rolling buffer
         */
        FilterDifferentiator2(unsigned int historySize = 100) :
            _deltaTime(0.1),
            _filterLowpass(),
            _isInitialized(false),
            _head(0),
            _bufferIn(),
            _time(0.0)
        {
            if (historySize < 1) {
                throw std::logic_error(
                    "inria::FilterDifferentiator: Invalid history size");
            }
            _bufferIn.resize(historySize+1);
            reset();
        }

        /**
         * @return read or write and read direct 
         * access to the lowpass cutoff frequency.
         */
        const double& cutoffFrequency() const
        {
            return _filterLowpass.cutoffFrequency();
        }
        double& cutoffFrequency()
        {
            return _filterLowpass.cutoffFrequency();
        }
        
        /**
         * @return read or write and read direct 
         * access to the maximum time difference
         */
        const double& timeDelta() const
        {
            return _deltaTime;
        }
        double& timeDelta()
        {
            return _deltaTime;
        }

        /**
         * Update the internal filtered 
         * state with in input value.
         *
         * @param value The new input value 
         * to be differentiated.
         */
        void update(const T& input, double dt)
        {
            //Initialization if needed
            if (!_isInitialized) {
                reset(input);
            }

            //Update internal time
            _time += dt;

            //Append the new value to 
            //input rolling buffer
            _bufferIn[rollingIndex(0)] = std::make_pair(_time, input);

            //Find the old data index with 
            //at most _deltaTime time difference
            size_t index = _bufferIn.size()-1;
            for (size_t i=1;i<_bufferIn.size()-1;i++) {
                if (
                    _bufferIn[rollingIndex(i)].first <= _bufferIn[rollingIndex(i+1)].first ||
                    _bufferIn[rollingIndex(0)].first - _bufferIn[rollingIndex(i)].first >= _deltaTime
                ) {
                    index = i;
                    break;
                }
            }
            //Compute the diff
            T diff = 
                (1.0/(_bufferIn[rollingIndex(0)].first - _bufferIn[rollingIndex(index)].first)) *
                (_bufferIn[rollingIndex(0)].second - _bufferIn[rollingIndex(index)].second);
            //Update the lowpass filter
            _filterLowpass.update(diff, dt);

            //Update rolling buffer index
            _head = (_head+1) % _bufferIn.size();
        }
        
        /**
         * @return read access to filtered 
         * differentiated value.
         */
        const T& value() const
        {
            return _filterLowpass.value();
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
            _head = 0;
            _time = 0.0;
            for (size_t i=0;i<_bufferIn.size();i++) {
                _bufferIn[i] = std::make_pair(0.0, value);
            }
        }

    private:

        /**
         * Maximum time difference 
         * for finite difference lookup
         */
        double _deltaTime;

        /**
         * Internal lowpass filter
         */
        FilterExponential<T> _filterLowpass;

        /**
         * If false, the current state is 
         * not yet initialized and should be
         * set at next update() call.
         */
        bool _isInitialized;

        /**
         * Begin index inside rolling buffer.
         * Index to the next cell to write.
         */
        int _head;
        
        /**
         * Rolling buffer for 
         * pair of time and input values
         */
        std::vector<std::pair<double, T>> _bufferIn;

        /**
         * Internal time counter
         */
        double _time;
        
        /**
         * Utility function computing buffer 
         * index in rolling buffer.
         *
         * @param pos Expected position relative to head.
         * @return the index in rolling buffer.
         */
        size_t rollingIndex(int pos) const
        {
            int size = _bufferIn.size();
            int index = _head - pos;
            while (index < 0) {
                index += size;
            }
            return index % size;
        }
};

}

#endif

