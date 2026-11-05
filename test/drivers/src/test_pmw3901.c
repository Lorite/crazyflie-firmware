// File under test pmw3901.c (Lorite #171: raw frame capture)
#include "pmw3901.h"

#include <string.h>
#include "unity.h"
#include "mock_deck_spi.h"
#include "mock_deck_digital.h"
#include "mock_sleepus.h"
#include "mock_usec_time.h"

#include "freertosMocks.h"

// The Flow deck chip select (DECK_GPIO_IO3 lives in deck_constants.c, which is not linked here)
static const deckPin_t TEST_CS_PIN = {.id = 6};

// A simulated PMW3901 behind the mocked SPI. Each register access is two
// spiExchange() calls: the address byte (MSB set for a write), then the data byte.

#define MAX_STREAM 8000
#define MAX_WRITES 256

static uint8_t stream[MAX_STREAM];   // what reads of 0x58 return, in order
static int streamLen;
static int streamPos;
static uint8_t streamTail;           // returned once the stream is used up

static uint8_t writeReg[MAX_WRITES];
static uint8_t writeVal[MAX_WRITES];
static int nWrites;

static bool expectData;
static uint8_t currentReg;
static bool currentIsWrite;
static uint64_t fakeTimeUs;

static uint8_t truth[PMW3901_FRAME_PIXELS];

static bool spiExchangeCallback(size_t length, const uint8_t *data_tx, uint8_t *data_rx, int cmock_num_calls)
{
  (void)cmock_num_calls;
  TEST_ASSERT_EQUAL(1, length);
  if (!expectData) {
    currentIsWrite = (data_tx[0] & 0x80u) != 0;
    currentReg = data_tx[0] & 0x7Fu;
    expectData = true;
    return true;
  }

  expectData = false;
  if (currentIsWrite) {
    TEST_ASSERT_TRUE(nWrites < MAX_WRITES);
    writeReg[nWrites] = currentReg;
    writeVal[nWrites] = data_tx[0];
    nWrites++;
    return true;
  }

  if (currentReg == 0x00) {
    data_rx[0] = 0x49;  // chip id
  } else if (currentReg == 0x5F) {
    data_rx[0] = 0xB6;  // inverse chip id
  } else if (currentReg == 0x58) {
    data_rx[0] = (streamPos < streamLen) ? stream[streamPos++] : streamTail;
  } else {
    data_rx[0] = 0;
  }
  return true;
}

static uint64_t usecTimestampCallback(int cmock_num_calls)
{
  (void)cmock_num_calls;
  fakeTimeUs += 10;
  return fakeTimeUs;
}

static void push(uint8_t value)
{
  TEST_ASSERT_TRUE(streamLen < MAX_STREAM);
  stream[streamLen++] = value;
}

// The sensor's encoding: tag 01 carries pixel bits 7:2 in bits 5:0, tag 10
// carries pixel bits 1:0 in bits 3:2, tags 00 and 11 carry no data.
static void pushPixel(uint8_t pixel)
{
  push(0x40u | (uint8_t)(pixel >> 2));
  push(0x80u | (uint8_t)((pixel & 0x03u) << 2));
}

void setUp(void)
{
  streamLen = 0;
  streamPos = 0;
  streamTail = 0x00;
  nWrites = 0;
  expectData = false;
  fakeTimeUs = 0;

  spiExchange_StubWithCallback(spiExchangeCallback);
  usecTimestamp_StubWithCallback(usecTimestampCallback);
  spiBeginTransaction_Ignore();
  spiEndTransaction_Ignore();
  spiBegin_Ignore();
  digitalWrite_Ignore();
  pinMode_Ignore();
  sleepus_Ignore();

  for (int i = 0; i < PMW3901_FRAME_PIXELS; i++) {
    truth[i] = (uint8_t)((i * 37 + 11) & 0xFF);
  }
}

void tearDown(void) {}

void testThatReinitFailsBeforeInit()
{
  TEST_ASSERT_FALSE(pmw3901Reinit(TEST_CS_PIN));
  TEST_ASSERT_EQUAL(0, nWrites);
}

void testThatEnableFrameCaptureWritesTheArduinoDriverSequence()
{
  // Bitcraze_PMW3901 enableFrameBuffer(), commit 374d488
  const uint8_t expectedReg[] = {0x7F, 0x41, 0x4C, 0x7F, 0x6A, 0x7F, 0x55, 0x40, 0x4D, 0x70, 0x58};
  const uint8_t expectedVal[] = {0x07, 0x1D, 0x00, 0x08, 0x38, 0x00, 0x04, 0x80, 0x11, 0x00, 0xFF};

  pmw3901EnableFrameCapture(TEST_CS_PIN);

  TEST_ASSERT_EQUAL(sizeof(expectedReg), nWrites);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedReg, writeReg, sizeof(expectedReg));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedVal, writeVal, sizeof(expectedVal));
}

void testThatReadFrameAssemblesEveryPixelAndSkipsInvalidReads()
{
  push(0xC0);  // busy (tag 11) twice before the frame starts
  push(0xC0);
  push(0x00);  // first status read that is not busy
  int invalid = 1;
  for (int i = 0; i < PMW3901_FRAME_PIXELS; i++) {
    if (i % 5 == 0) {
      push(0x00);  // no data yet
      invalid++;
    }
    push(0x40u | (uint8_t)(truth[i] >> 2));
    if (i % 7 == 0) {
      push(0xC0);  // no data between the two halves
      invalid++;
    }
    push(0x80u | (uint8_t)((truth[i] & 0x03u) << 2));
  }

  uint8_t frame[PMW3901_FRAME_PIXELS];
  memset(frame, 0, sizeof(frame));
  pmw3901FrameStats_t stats;

  const bool ok = pmw3901ReadFrame(TEST_CS_PIN, frame, 50, 10000000, &stats);

  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL(PMW3901_FRAME_PIXELS, stats.pixels);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(truth, frame, PMW3901_FRAME_PIXELS);
  // The 0x00 after the two busy reads ends the wait, so it is not counted
  TEST_ASSERT_EQUAL(invalid - 1, stats.invalidReads);
  TEST_ASSERT_EQUAL(0, stats.orderErrors);
}

void testThatTheLowTwoBitsAreKept()
{
  // Pixels whose value depends only on the low two bits. The Arduino formula
  // (a << 2) + (b & 0x0C) gets these wrong, the Pimoroni formula gets them right.
  push(0x00);
  const uint8_t values[] = {0x00, 0x01, 0x02, 0x03, 0xFC, 0xFD, 0xFE, 0xFF};
  for (int i = 0; i < PMW3901_FRAME_PIXELS; i++) {
    truth[i] = values[i % sizeof(values)];
    pushPixel(truth[i]);
  }

  uint8_t frame[PMW3901_FRAME_PIXELS];
  pmw3901FrameStats_t stats;
  TEST_ASSERT_TRUE(pmw3901ReadFrame(TEST_CS_PIN, frame, 50, 10000000, &stats));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(truth, frame, PMW3901_FRAME_PIXELS);
}

void testThatOrderErrorsAreCountedAndDoNotShiftTheFrame()
{
  push(0x00);
  push(0x80);         // a low half without a high half: ignored, counted
  push(0x40 | 0x3F);  // a high half that is overwritten by the next one: counted
  for (int i = 0; i < PMW3901_FRAME_PIXELS; i++) {
    pushPixel(truth[i]);
  }

  uint8_t frame[PMW3901_FRAME_PIXELS];
  pmw3901FrameStats_t stats;
  TEST_ASSERT_TRUE(pmw3901ReadFrame(TEST_CS_PIN, frame, 50, 10000000, &stats));
  TEST_ASSERT_EQUAL(2, stats.orderErrors);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(truth, frame, PMW3901_FRAME_PIXELS);
}

void testThatReadFrameTimesOutWhenNoPixelsArrive()
{
  push(0x00);
  streamTail = 0x00;  // never any data

  uint8_t frame[PMW3901_FRAME_PIXELS];
  pmw3901FrameStats_t stats;
  const bool ok = pmw3901ReadFrame(TEST_CS_PIN, frame, 50, 5000, &stats);

  TEST_ASSERT_FALSE(ok);
  TEST_ASSERT_EQUAL(0, stats.pixels);
  TEST_ASSERT_TRUE(stats.captureUs > 5000);
  TEST_ASSERT_TRUE(stats.captureUs < 6000);
}

void testThatReadFrameTimesOutWhileTheSensorStaysBusy()
{
  streamTail = 0xC0;

  uint8_t frame[PMW3901_FRAME_PIXELS];
  pmw3901FrameStats_t stats;
  TEST_ASSERT_FALSE(pmw3901ReadFrame(TEST_CS_PIN, frame, 50, 5000, &stats));
  TEST_ASSERT_EQUAL(0, stats.pixels);
}

void testThatReinitAfterInitResetsAndRewritesTheMotionRegisters()
{
  TEST_ASSERT_TRUE(pmw3901Init(TEST_CS_PIN));
  const int writesAtInit = nWrites;
  TEST_ASSERT_EQUAL(0x3A, writeReg[0]);  // power-on reset first
  TEST_ASSERT_EQUAL(0x5A, writeVal[0]);

  nWrites = 0;
  TEST_ASSERT_TRUE(pmw3901Reinit(TEST_CS_PIN));

  // The same register set as at boot, starting with the power-on reset
  TEST_ASSERT_EQUAL(writesAtInit, nWrites);
  TEST_ASSERT_EQUAL(0x3A, writeReg[0]);
  TEST_ASSERT_EQUAL(0x5A, writeVal[0]);
  // InitRegisters() ends with 0x7F=0x00, 0x5A=0x10, 0x54=0x00
  TEST_ASSERT_EQUAL(0x54, writeReg[nWrites - 1]);
  TEST_ASSERT_EQUAL(0x00, writeVal[nWrites - 1]);
  TEST_ASSERT_EQUAL(0x5A, writeReg[nWrites - 2]);
  TEST_ASSERT_EQUAL(0x10, writeVal[nWrites - 2]);
}
