#include "ros_map_edit/map_grid_utils.h"

#include <cstddef>
#include <cstdint>
#include <limits>

using ros_map_edit::MapBrushMode;

static_assert(ros_map_edit::brushModeForMouseButton(true) == MapBrushMode::OCCUPIED,
              "left click must select the occupied brush");
static_assert(ros_map_edit::occupancyValue(ros_map_edit::brushModeForMouseButton(true)) ==
                  static_cast<std::int8_t>(100),
              "left click must paint occupied cells");

static_assert(ros_map_edit::brushModeForMouseButton(false) == MapBrushMode::FREE,
              "right click must select the free brush");
static_assert(ros_map_edit::occupancyValue(ros_map_edit::brushModeForMouseButton(false)) ==
                  static_cast<std::int8_t>(0),
              "right click must paint free cells");
static_assert(ros_map_edit::occupancyValue(MapBrushMode::UNKNOWN) == static_cast<std::int8_t>(-1),
              "the unknown brush must paint unknown cells");

constexpr std::size_t kWidth = 4;
constexpr std::size_t kHeight = 3;
static_assert(ros_map_edit::pgmPixelToMapIndex(0, kWidth, kHeight) == 8,
              "top-left PGM pixel must map to the last grid row");
static_assert(ros_map_edit::pgmPixelToMapIndex(3, kWidth, kHeight) == 11,
              "top-right PGM pixel must map to the last grid row");
static_assert(ros_map_edit::pgmPixelToMapIndex(5, kWidth, kHeight) == 5,
              "middle PGM pixels must remain on the middle grid row");
static_assert(ros_map_edit::pgmPixelToMapIndex(8, kWidth, kHeight) == 0,
              "bottom-left PGM pixel must map to the first grid row");
static_assert(ros_map_edit::pgmPixelToMapIndex(11, kWidth, kHeight) == 3,
              "bottom-right PGM pixel must map to the first grid row");

static_assert(ros_map_edit::hasPositiveDimensions(4, 3),
              "positive map dimensions must be accepted");
static_assert(!ros_map_edit::hasPositiveDimensions(0, 3),
              "zero map width must be rejected");
static_assert(!ros_map_edit::hasPositiveDimensions(4, 0),
              "zero map height must be rejected");
static_assert(!ros_map_edit::hasPositiveDimensions(-1, 3),
              "negative map width must be rejected before casting");
static_assert(!ros_map_edit::hasPositiveDimensions(4, -1),
              "negative map height must be rejected before casting");

static_assert(ros_map_edit::pixelCountFits(kWidth, kHeight),
              "ordinary map dimensions must fit in size_t");
static_assert(!ros_map_edit::pixelCountFits(std::numeric_limits<std::size_t>::max(), 2),
              "pixel-count multiplication overflow must be rejected");

int main()
{
  return 0;
}
