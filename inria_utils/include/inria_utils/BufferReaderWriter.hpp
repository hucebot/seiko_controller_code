#ifndef INRIA_UTILS_BUFFERREADERWRITER_HPP
#define INRIA_UTILS_BUFFERREADERWRITER_HPP

#include <atomic>

namespace inria {

/**
 * BufferReaderWriter
 *
 * Lock-free triple buffers
 * to share data from single reader to single writer 
 * thread using a push/pull semantic.
 * The writer can always update its own data instance
 * then export the data using push.
 * The reader can always read its own data 
 * instance then import new data using pull.
 *
 * Template parameters:
 * @param T Type of stored data
 */
template <typename T>
class BufferReaderWriter
{
    public:

        /**
         * Initialization with externally 
         * allocated data instances.
         * Forwarded arguments of constructor 
         * for T is given.
         */
        template <typename ... Args>
        BufferReaderWriter(Args&&... args) :
            _dataReader(new T(std::forward<Args>(args)...)),
            _dataWriter(new T(std::forward<Args>(args)...)),
            _dataShared(new T(std::forward<Args>(args)...))
        {
        }

        /**
         * Deallocation of internal data
         */
        ~BufferReaderWriter()
        {
            if (_dataReader != nullptr) {
                delete _dataReader;
            }
            if (_dataWriter != nullptr) {
                delete _dataWriter;
            }
            if (_dataShared != nullptr) {
                delete _dataShared;
            }
        }
        
        /**
         * Copy and assignment are disabled
         * to prevent memory management issues
         */
        BufferReaderWriter(const BufferReaderWriter&) = delete;
        BufferReaderWriter& operator=(const BufferReaderWriter&) = delete;

        /**
         * Reset all internal buffer with 
         * given value.
         * (NOT thread safe).
         */
        void reset(const T& value)
        {
            *_dataReader = value;
            *_dataWriter = value;
            *_dataShared.load() = value;
        }

        /**
         * Access to reader 
         * private data instance.
         * (Read access is used to reset the
         * internal buffer after reading it)
         */
        const T* getFromReader() const noexcept
        {
            return _dataReader;
        }
        T* getFromReader() noexcept
        {
            return _dataReader;
        }

        /**
         * Atomically swap reader and 
         * shared instances.
         * Import last pushed writer data.
         */
        void pullFromReader() noexcept
        {
            T* tmp = _dataShared.exchange(_dataReader);
            _dataReader = tmp;
        }

        /**
         * Access to writer
         * private data instance
         */
        T* getFromWriter() noexcept
        {
            return _dataWriter;
        }

        /**
         * Atomically swap reader and 
         * shared instances.
         * Export currently written data instance.
         */
        void pushFromWriter() noexcept
        {
            T* tmp = _dataShared.exchange(_dataWriter);
            _dataWriter = tmp;
        }

    private:

        /**
         * The allocated data privately owned 
         * by the reader and the writer
         */
        T* _dataReader;
        T* _dataWriter;

        /**
         * The allocated data shared 
         * and waiting to be pulled
         */
        std::atomic<T*> _dataShared;
};

}

#endif

