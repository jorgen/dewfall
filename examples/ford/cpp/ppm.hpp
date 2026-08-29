/* Binary PPM (P6) reader. The Ford images are 1616x616 per camera, 8 bits per channel. */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ford
{

struct image_t
{
  int32_t width = 0;
  int32_t height = 0;
  std::vector<uint8_t> rgb; // width * height * 3, row-major

  [[nodiscard]] bool contains(int32_t x, int32_t y) const
  {
    return x >= 0 && y >= 0 && x < width && y < height;
  }
  [[nodiscard]] const uint8_t *at(int32_t x, int32_t y) const
  {
    return rgb.data() + (size_t(y) * size_t(width) + size_t(x)) * 3;
  }
};

// P6 only. The header is whitespace-separated tokens with '#' comments, and the Ford files put all
// four on one line while the FULL images put the magic on its own -- so it has to be tokenized,
// not read line by line.
bool ppm_read(const std::string &path, image_t &out, std::string &error);

} // namespace ford
