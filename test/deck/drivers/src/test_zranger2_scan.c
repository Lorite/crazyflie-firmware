// File under test zranger2_scan.c (Lorite #171: multizone ROI scan)
#include "zranger2_scan.h"

#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

static void assertRoi(uint8_t tlx, uint8_t tly, uint8_t brx, uint8_t bry, const zscanRoi_t *roi)
{
  TEST_ASSERT_EQUAL_UINT8(tlx, roi->topLeftX);
  TEST_ASSERT_EQUAL_UINT8(tly, roi->topLeftY);
  TEST_ASSERT_EQUAL_UINT8(brx, roi->botRightX);
  TEST_ASSERT_EQUAL_UINT8(bry, roi->botRightY);
}

void testThatTheTwoByTwoGridIsTheFourQuadrants()
{
  zscanRoi_t roi;
  zscanZoneRoi(2, 0, &roi);
  assertRoi(0, 15, 7, 8, &roi);
  zscanZoneRoi(2, 1, &roi);
  assertRoi(8, 15, 15, 8, &roi);
  zscanZoneRoi(2, 2, &roi);
  assertRoi(0, 7, 7, 0, &roi);
  zscanZoneRoi(2, 3, &roi);
  assertRoi(8, 7, 15, 0, &roi);
}

void testThatTheFourByFourGridTilesTheArrayWithoutOverlap()
{
  int covered[16][16] = {{0}};
  for (uint8_t zone = 0; zone < 16; zone++) {
    zscanRoi_t roi;
    zscanZoneRoi(4, zone, &roi);
    TEST_ASSERT_EQUAL_UINT8(3, roi.botRightX - roi.topLeftX);
    TEST_ASSERT_EQUAL_UINT8(3, roi.topLeftY - roi.botRightY);
    for (int x = roi.topLeftX; x <= roi.botRightX; x++) {
      for (int y = roi.botRightY; y <= roi.topLeftY; y++) {
        covered[x][y]++;
      }
    }
  }
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      TEST_ASSERT_EQUAL(1, covered[x][y]);
    }
  }
}

void testThatEveryZoneIsAValidStRoi()
{
  // The ST API needs coordinates in 0..15, TopLeftX <= BotRightX,
  // TopLeftY >= BotRightY, and at least 4 x 4 SPADs (width and height >= 3)
  const uint8_t grids[] = {2, 3, 4};
  for (unsigned g = 0; g < sizeof(grids); g++) {
    const uint8_t grid = grids[g];
    for (uint8_t zone = 0; zone < grid * grid; zone++) {
      zscanRoi_t roi;
      zscanZoneRoi(grid, zone, &roi);
      TEST_ASSERT_TRUE(roi.botRightX <= 15);
      TEST_ASSERT_TRUE(roi.topLeftY <= 15);
      TEST_ASSERT_TRUE(roi.topLeftX <= roi.botRightX);
      TEST_ASSERT_TRUE(roi.topLeftY >= roi.botRightY);
      TEST_ASSERT_TRUE(roi.botRightX - roi.topLeftX >= 3);
      TEST_ASSERT_TRUE(roi.topLeftY - roi.botRightY >= 3);
    }
  }
}

void testThatTheThreeByThreeGridReachesBothEdges()
{
  zscanRoi_t roi;
  zscanZoneRoi(3, 0, &roi);
  assertRoi(0, 15, 5, 10, &roi);
  zscanZoneRoi(3, 4, &roi);
  assertRoi(5, 10, 10, 5, &roi);
  zscanZoneRoi(3, 8, &roi);
  assertRoi(10, 5, 15, 0, &roi);
}

void testThatTheScanOffAlwaysUsesTheFullArray()
{
  uint8_t nextZone = 0;
  zscanSlot_t slot = {.full = true, .zone = 0};
  for (int i = 0; i < 10; i++) {
    slot = zscanChooseNext(slot, 0, true, &nextZone);
    TEST_ASSERT_TRUE(slot.full);
  }
  TEST_ASSERT_EQUAL_UINT8(0, nextZone);
}

void testThatInterleaveAlternatesFullAndEveryZoneInOrder()
{
  uint8_t nextZone = 0;
  zscanSlot_t slot = {.full = true, .zone = 0};
  for (int lap = 0; lap < 2; lap++) {
    for (uint8_t zone = 0; zone < 4; zone++) {
      slot = zscanChooseNext(slot, 2, true, &nextZone);
      TEST_ASSERT_FALSE(slot.full);
      TEST_ASSERT_EQUAL_UINT8(zone, slot.zone);
      slot = zscanChooseNext(slot, 2, true, &nextZone);
      TEST_ASSERT_TRUE(slot.full);
    }
  }
}

void testThatWithoutInterleaveOnlyZonesAreRanged()
{
  uint8_t nextZone = 0;
  zscanSlot_t slot = {.full = true, .zone = 0};
  for (int i = 0; i < 20; i++) {
    slot = zscanChooseNext(slot, 3, false, &nextZone);
    TEST_ASSERT_FALSE(slot.full);
    TEST_ASSERT_EQUAL_UINT8(i % 9, slot.zone);
  }
}

void testThatASmallerGridRestartsTheZoneIndex()
{
  uint8_t nextZone = 12;  // from a 4 x 4 scan
  zscanSlot_t slot = {.full = true, .zone = 0};
  slot = zscanChooseNext(slot, 2, true, &nextZone);
  TEST_ASSERT_FALSE(slot.full);
  TEST_ASSERT_EQUAL_UINT8(0, slot.zone);
}
