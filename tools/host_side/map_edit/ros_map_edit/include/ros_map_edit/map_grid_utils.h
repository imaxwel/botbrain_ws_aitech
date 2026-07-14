#ifndef ROS_MAP_EDIT_MAP_GRID_UTILS_H
#define ROS_MAP_EDIT_MAP_GRID_UTILS_H

#include <cstddef>
#include <cstdint>
#include <limits>

namespace ros_map_edit
{

enum class MapBrushMode
{
  FREE,
  OCCUPIED,
  UNKNOWN
};

constexpr MapBrushMode brushModeForMouseButton(bool left_button)
{
  return left_button ? MapBrushMode::OCCUPIED : MapBrushMode::FREE;
}

constexpr std::int8_t occupancyValue(MapBrushMode mode)
{
  return mode == MapBrushMode::OCCUPIED ? static_cast<std::int8_t>(100) :
         mode == MapBrushMode::FREE ? static_cast<std::int8_t>(0) :
                                      static_cast<std::int8_t>(-1);
}

constexpr bool hasPositiveDimensions(int width, int height)
{
  return width > 0 && height > 0;
}

constexpr bool pixelCountFits(std::size_t width, std::size_t height)
{
  return width == 0 || height <= std::numeric_limits<std::size_t>::max() / width;
}

// Preconditions: dimensions are nonzero and pgm_pixel_index is in range.
constexpr std::size_t pgmPixelToMapIndex(std::size_t pgm_pixel_index,
                                         std::size_t width,
                                         std::size_t height)
{
  const std::size_t pgm_row = pgm_pixel_index / width;
  const std::size_t column = pgm_pixel_index % width;
  return (height - 1 - pgm_row) * width + column;
}

}  // namespace ros_map_edit

#endif  // ROS_MAP_EDIT_MAP_GRID_UTILS_H
