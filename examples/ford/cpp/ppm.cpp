#include "ppm.hpp"

#include <cstdio>

#include <fmt/format.h>

namespace ford
{
namespace
{
bool is_space(uint8_t c)
{
  return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f';
}
} // namespace

bool ppm_read(const std::string &path, image_t &out, std::string &error)
{
  std::vector<uint8_t> bytes;
  FILE *f = fopen(path.c_str(), "rb");
  if (!f)
  {
    error = fmt::format("cannot open {}", path);
    return false;
  }
  fseek(f, 0, SEEK_END);
  const long len = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (len <= 0)
  {
    fclose(f);
    error = fmt::format("{} is empty", path);
    return false;
  }
  bytes.resize(size_t(len));
  const size_t got = fread(bytes.data(), 1, bytes.size(), f);
  fclose(f);
  if (got != bytes.size())
  {
    error = fmt::format("short read on {}", path);
    return false;
  }

  size_t i = 0;
  std::string tokens[4];
  for (int t = 0; t < 4;)
  {
    while (i < bytes.size() && is_space(bytes[i]))
      i++;
    if (i < bytes.size() && bytes[i] == '#')
    {
      while (i < bytes.size() && bytes[i] != '\n')
        i++;
      continue;
    }
    const size_t start = i;
    while (i < bytes.size() && !is_space(bytes[i]))
      i++;
    if (start == i)
    {
      error = fmt::format("{} has a truncated PPM header", path);
      return false;
    }
    tokens[t++].assign(reinterpret_cast<const char *>(bytes.data() + start), i - start);
  }
  i++; // exactly one whitespace byte separates the header from the data

  if (tokens[0] != "P6")
  {
    error = fmt::format("{} is '{}', not a binary P6 PPM", path, tokens[0]);
    return false;
  }
  out.width = atoi(tokens[1].c_str());
  out.height = atoi(tokens[2].c_str());
  const int max_value = atoi(tokens[3].c_str());
  if (out.width <= 0 || out.height <= 0)
  {
    error = fmt::format("{} has dimensions {}x{}", path, out.width, out.height);
    return false;
  }
  if (max_value != 255)
  {
    error = fmt::format("{} has max value {}; only 8-bit PPM is supported", path, max_value);
    return false;
  }
  const size_t need = size_t(out.width) * size_t(out.height) * 3;
  if (bytes.size() - i < need)
  {
    error = fmt::format("{} holds {} pixel bytes, {}x{} needs {}", path, bytes.size() - i, out.width, out.height, need);
    return false;
  }
  out.rgb.assign(bytes.begin() + long(i), bytes.begin() + long(i + need));
  return true;
}

} // namespace ford
