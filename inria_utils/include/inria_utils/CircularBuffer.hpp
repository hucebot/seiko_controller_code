#ifndef INRIA_UTILS_CIRCULARBUFFER_HPP
#define INRIA_UTILS_CIRCULARBUFFER_HPP

namespace inria {

/**
 * CircularBuffer
 *
 * Template type circular buffer
 * with static memory allocation.
 * Real time safe but NOT thread safe.
 */
template <typename T, size_t SIZE>
class CircularBuffer
{
    public:

        /**
         * Default initialization
         */
        CircularBuffer() :
            _buffer(),
            _head(0),
            _size(0),
            _neutral()
        {
            clear();
        }

        /**
         * @return the number of written 
         * element in the buffer
         */
        size_t size() const
        {
            return _size;
        }

        /**
         * Reset internal buffer to empty
         */
        void clear()
        {
            _head = 0;
            _size = 0;
        }

        /**
         * Append given data element into 
         * the circular buffer
         */
        void append(const T& value)
        {
            _buffer[_head] = value;
            if (_size < SIZE) {
                _size++;
            }
            _head++;
            if (_head >= SIZE) {
                _head = 0;
            }
        }

        /**
         * @return the stored value indexed backward 
         * in time by its index (from 0 to size()-1).
         */
        const T& get(size_t index) const
        {
            if (_size == 0) {
                //Return default value if the buffer
                //is not yet initialized
                return _neutral;
            }
            if (index >= _size) {
                //Return the oldest element if index 
                //is outside available data
                index = _size-1;
            }
            if (index+1 <= _head) {
                return _buffer[_head-1-index];
            } else {
                return _buffer[SIZE-(index+1-_head)];
            }
        }

    private:

        /**
         * Statically allocated data 
         * circular buffer
         */
        T _buffer[SIZE];

        /**
         * Index to next buffer element 
         * to be written
         */
        size_t _head;

        /**
         * Current buffer used 
         * elements size
         */
        size_t _size;

        /**
         * Default neutral value used in get() 
         * for uninitialized buffer
         */
        T _neutral;
};

}

#endif

