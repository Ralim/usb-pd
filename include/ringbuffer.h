#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <string.h>
/*
 * create a trivial ringbuffer with space for up to size elements.
 */
template <typename T, size_t size> class ringbuffer {
public:
  explicit ringbuffer() : begin(0), end(0), wrap(false) {}

  void push(const T *data) {

    memcpy(buffer + end, data, sizeof(T));
    // If going to wrap, push start along to maintain order
    if (begin == end && wrap) {
      begin = (begin + 1) % size;
    }
    end = (end + 1) % size;
    if (begin == end) {
      wrap = true;
    }
  }
  // Give null to just drop the data
  void pop(T *dest) {
    if (getOccupied() == 0) {
      return;
    }

    if (dest) {
      memcpy(dest, buffer + begin, sizeof(T));
    }
    begin = (begin + 1) % size;
    if (wrap && (begin == 0)) {
      wrap = false;
    }
  }
  // Returns number of objects queued in the buffer
  size_t getOccupied() const {
    if (end == begin) {
      return wrap ? size : 0;
    } else if (end > begin) {
      return end - begin;
    } else {
      return size + end - begin;
    }
  }

  size_t getFree() const { return size - getOccupied(); }
  // Clear the entire buffer
  void flush() {
    wrap  = false;
    begin = end = 0;
  }

private:
  T      buffer[size];
  size_t begin;
  size_t end;
  bool   wrap;
};

#endif // RINGBUFFER_H
