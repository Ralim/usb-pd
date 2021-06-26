#include "CppUTest/TestHarness.h"
#include "ringbuffer.h"
#include <cstring>
#include <iostream>
#include <stdint.h>
TEST_GROUP(RINGBUFFER){};
TEST(RINGBUFFER, BasicCount) {
  ringbuffer<int, 10> buffer;
  for (int i = 0; i < 10; i++) {
    buffer.push(&i);
  }
  for (int i = 0; i < 10; i++) {
    int x = 0;
    buffer.pop(&x);
    CHECK_EQUAL(i, x);
  }
  CHECK_EQUAL(0, buffer.getOccupied());

  int x = -10;
  buffer.pop(&x);
  CHECK_EQUAL(-10, x);
}

TEST(RINGBUFFER, Wrap) {
  ringbuffer<int, 10> buffer;
  for (int i = 0; i < 15; i++) {
    buffer.push(&i);
  }
  CHECK_EQUAL(10, buffer.getOccupied());
  for (int i = 5; i < 15; i++) {
    int x = 0;
    buffer.pop(&x);
    CHECK_EQUAL(i, x);
  }
  CHECK_EQUAL(0, buffer.getOccupied());

  int x = -10;
  buffer.pop(&x);
  CHECK_EQUAL(-10, x);
}

TEST(RINGBUFFER, Flush) {
  ringbuffer<int, 10> buffer;
  for (int i = 0; i < 15; i++) {
    buffer.push(&i);
  }
  buffer.flush();
  CHECK_EQUAL(0, buffer.getOccupied());

  int x = -10;
  buffer.pop(&x);
  CHECK_EQUAL(-10, x);
}
