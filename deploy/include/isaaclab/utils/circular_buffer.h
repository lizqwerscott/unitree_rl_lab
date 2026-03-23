#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <vector>
#include <cstddef>
#include <cstring>
#include <algorithm>
#include <stdexcept>

/**
 * @brief A circular buffer with fixed length and filled with a default value.
 *
 * This class implements a circular buffer that stores elements in a fixed-size buffer.
 * When the buffer is full, new elements overwrite the oldest ones.
 *
 * @tparam T The type of elements stored in the buffer
 */
template<typename T>
class CircularBuffer
{
public:
    /**
     * @brief Construct a new CircularBuffer object
     *
     * @param length The fixed length of the buffer
     */
    explicit CircularBuffer(size_t length)
        : _buffer(length),
          _length(length),
          _num_pushes(0),
          _head(0)
    {
        if (length == 0) {
            throw std::invalid_argument("CircularBuffer length must be greater than 0");
        }
    }

    /**
     * @brief Append a value to the buffer
     *
     * If the buffer is full, the oldest value will be overwritten.
     *
     * @param value The value to append
     */
    void append(const T& value)
    {
        if (_num_pushes == 0) {
            // First push: fill entire buffer with the value
            std::fill(_buffer.begin(), _buffer.end(), value);
        } else {
            // Normal push: overwrite oldest element
            _buffer[_head] = value;
        }

        // Move head to next position (circular)
        _head = (_head + 1) % _length;
        _num_pushes++;
    }

    /**
     * @brief Get the current buffer contents
     *
     * @return const std::vector<T>& Reference to the buffer contents
     */
    const std::vector<T>& buffer() const
    {
        return _buffer;
    }

    std::vector<T> buffer_index(std::vector<int> &indexs) const
    {
        std::vector<T> result(indexs.size());

        for (int i = 0; i < indexs.size(); ++i) {
			int index = _buffer.size() + indexs[i];
			if (index < 0 || index >= _buffer.size()) {
				printf("Error: %d index, %ld\n", index, _buffer.size());
                throw std::out_of_range("Index out of range in buffer_index");
			}
            result.push_back(_buffer[index]);
        }

        return result;
    }

    /**
     * @brief Get the buffer in chronological order (oldest to newest)
     *
     * @return std::vector<T> Buffer contents in chronological order
     */
    std::vector<T> buffer_chronological() const
    {
        if (_num_pushes == 0) {
            return _buffer;
        }

        std::vector<T> result(_length);

        // Start from the oldest element (head points to next write position)
        size_t start_idx = _head;  // head points to next write, so oldest is at head

        for (size_t i = 0; i < _length; ++i) {
            size_t idx = (start_idx + i) % _length;
            result[i] = _buffer[idx];
        }

        return result;
    }

    /**
     * @brief Reset the buffer to all zeros
     */
    void reset()
    {
        std::fill(_buffer.begin(), _buffer.end(), T());
        _num_pushes = 0;
        _head = 0;
    }

    /**
     * @brief Get the number of pushes performed
     *
     * @return size_t Number of pushes
     */
    size_t num_pushes() const
    {
        return _num_pushes;
    }

    /**
     * @brief Check if the buffer is full
     *
     * @return true if buffer is full
     * @return false if buffer is not full
     */
    bool is_full() const
    {
        return _num_pushes >= _length;
    }

    /**
     * @brief Get the length of the buffer
     *
     * @return size_t Buffer length
     */
    size_t length() const
    {
        return _length;
    }

    /**
     * @brief Get the most recent element
     *
     * @return const T& Reference to the most recent element
     * @throws std::runtime_error if buffer is empty
     */
    const T& latest() const
    {
        if (_num_pushes == 0) {
            throw std::runtime_error("CircularBuffer is empty");
        }

        // Most recent element is at position before head (circular)
        size_t latest_idx = (_head == 0) ? _length - 1 : _head - 1;
        return _buffer[latest_idx];
    }

    /**
     * @brief Get the oldest element
     *
     * @return const T& Reference to the oldest element
     * @throws std::runtime_error if buffer is empty
     */
    const T& oldest() const
    {
        if (_num_pushes == 0) {
            throw std::runtime_error("CircularBuffer is empty");
        }

        // Oldest element is at head position (head points to next write)
        return _buffer[_head];
    }

private:
    std::vector<T> _buffer;    ///< The underlying buffer storage
    size_t _length;            ///< Fixed length of the buffer
    size_t _num_pushes;        ///< Total number of pushes performed
    size_t _head;              ///< Index of next write position (points to oldest element when full)
};

#endif
