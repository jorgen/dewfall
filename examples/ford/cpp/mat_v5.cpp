#include "mat_v5.hpp"

#include <cstdio>
#include <cstring>

#include <fmt/format.h>
#include <zlib-ng.h>

namespace ford
{
namespace
{

// MAT-file data types (the type of the BYTES, not of the array).
enum mi_type_t : uint32_t
{
  mi_int8 = 1,
  mi_uint8 = 2,
  mi_int16 = 3,
  mi_uint16 = 4,
  mi_int32 = 5,
  mi_uint32 = 6,
  mi_single = 7,
  mi_double = 9,
  mi_int64 = 12,
  mi_uint64 = 13,
  mi_matrix = 14,
  mi_compressed = 15,
  mi_utf8 = 16,
};

// Array classes (what the array MEANS).
enum mx_class_t : uint8_t
{
  mx_cell = 1,
  mx_struct = 2,
  mx_object = 3,
  mx_char = 4,
  mx_sparse = 5,
  mx_double = 6,
  mx_single = 7,
  mx_int8 = 8,
  mx_uint8 = 9,
  mx_int16 = 10,
  mx_uint16 = 11,
  mx_int32 = 12,
  mx_uint32 = 13,
  mx_int64 = 14,
  mx_uint64 = 15,
};

constexpr uint32_t k_header_size = 128;
constexpr uint32_t k_flag_complex = 0x0800;

// A bounds-checked cursor over the file bytes. Every read goes through it, because a .mat file is
// untrusted input whose every length field is attacker-controlled: a single unchecked size turns a
// malformed scan into a heap overread.
struct cursor_t
{
  const uint8_t *begin = nullptr;
  const uint8_t *end = nullptr;
  const uint8_t *at = nullptr;

  // Saturating, NOT `end - at`. The last element's 8-byte padding can legitimately run past the end
  // of the buffer, and an unsigned difference there wraps to ~2^64 -- so every subsequent bounds
  // check passes and the parser walks off the heap. That is exactly how this reader first crashed.
  [[nodiscard]] size_t remaining() const
  {
    return at >= end ? 0 : size_t(end - at);
  }
  [[nodiscard]] bool has(size_t n) const
  {
    return remaining() >= n;
  }
};

// One data element's tag. MATLAB has two encodings: the normal 8-byte (type, size) pair, and a
// "small data element" form that packs a size of 1-4 bytes into the upper half of the type word so
// the whole element fits in 8 bytes. Missing the small form does not fail loudly -- it reads a
// plausible-looking type and walks off into the middle of the data.
struct tag_t
{
  uint32_t type = 0;
  uint32_t size = 0;
  const uint8_t *payload = nullptr;
  bool small = false;
};

bool read_tag(cursor_t &c, tag_t &tag, std::string &error)
{
  if (!c.has(8))
  {
    error = "truncated: no room for a data element tag";
    return false;
  }
  uint32_t raw_type = 0;
  uint32_t raw_size = 0;
  memcpy(&raw_type, c.at, 4);
  memcpy(&raw_size, c.at + 4, 4);
  if ((raw_type >> 16) & 0xFFFFu)
  {
    tag.small = true;
    tag.type = raw_type & 0xFFFFu;
    tag.size = raw_type >> 16;
    tag.payload = c.at + 4;
  }
  else
  {
    tag.small = false;
    tag.type = raw_type;
    tag.size = raw_size;
    tag.payload = c.at + 8;
  }
  const size_t consumed = size_t(tag.payload - c.at) + tag.size;
  if (!c.has(consumed))
  {
    error = fmt::format("truncated: element of type {} claims {} bytes, {} remain", tag.type, tag.size, c.remaining());
    return false;
  }
  return true;
}

// Advance past an element. Normal elements are padded to an 8-byte boundary; small ones are not
// (their padding is already inside the 8 bytes the tag occupies).
void skip_element(cursor_t &c, const tag_t &tag)
{
  const size_t consumed = size_t(tag.payload - c.at) + tag.size;
  const size_t step = tag.small ? consumed : consumed + ((8 - tag.size % 8) % 8);
  // Clamped to end for the same reason remaining() saturates: the final element's padding is
  // allowed to be absent, so stepping over it can land past the buffer.
  c.at = step >= size_t(c.end - c.at) ? c.end : c.at + step;
}

size_t size_of_mi(uint32_t type)
{
  switch (type)
  {
  case mi_int8:
  case mi_uint8:
  case mi_utf8:
    return 1;
  case mi_int16:
  case mi_uint16:
    return 2;
  case mi_int32:
  case mi_uint32:
  case mi_single:
    return 4;
  case mi_double:
  case mi_int64:
  case mi_uint64:
    return 8;
  default:
    return 0;
  }
}

// Widen `count` elements of on-disk type `type` into doubles. THE function this reader exists for:
// see the header's note on class-vs-type.
template <typename T>
void widen_into(const uint8_t *src, size_t count, std::vector<double> &out)
{
  out.resize(count);
  for (size_t i = 0; i < count; i++)
  {
    T v;
    memcpy(&v, src + i * sizeof(T), sizeof(T));
    out[i] = double(v);
  }
}

bool read_numeric_payload(const tag_t &tag, size_t expected, std::vector<double> &out, std::string &error)
{
  const size_t element = size_of_mi(tag.type);
  if (element == 0)
  {
    error = fmt::format("unsupported numeric data type {}", tag.type);
    return false;
  }
  const size_t available = tag.size / element;
  if (available < expected)
  {
    error = fmt::format("numeric element holds {} values, dimensions call for {}", available, expected);
    return false;
  }
  switch (tag.type)
  {
  case mi_int8:
    widen_into<int8_t>(tag.payload, expected, out);
    break;
  case mi_uint8:
  case mi_utf8:
    widen_into<uint8_t>(tag.payload, expected, out);
    break;
  case mi_int16:
    widen_into<int16_t>(tag.payload, expected, out);
    break;
  case mi_uint16:
    widen_into<uint16_t>(tag.payload, expected, out);
    break;
  case mi_int32:
    widen_into<int32_t>(tag.payload, expected, out);
    break;
  case mi_uint32:
    widen_into<uint32_t>(tag.payload, expected, out);
    break;
  case mi_single:
    widen_into<float>(tag.payload, expected, out);
    break;
  case mi_double:
    widen_into<double>(tag.payload, expected, out);
    break;
  case mi_int64:
    widen_into<int64_t>(tag.payload, expected, out);
    break;
  case mi_uint64:
    widen_into<uint64_t>(tag.payload, expected, out);
    break;
  default:
    error = fmt::format("unsupported numeric data type {}", tag.type);
    return false;
  }
  return true;
}

bool read_matrix(cursor_t &c, mat_value_t &out, std::string &error);

// The four leading subelements every miMATRIX carries: array flags, dimensions, name. (The fourth
// is the payload, which varies by class and is read by the caller.)
bool read_matrix_prologue(cursor_t &c, uint8_t &cls, uint32_t &flags, std::vector<int32_t> &dims, std::string &name, std::string &error)
{
  tag_t tag;
  if (!read_tag(c, tag, error))
    return false;
  if (tag.size < 8)
  {
    error = "array flags subelement is too small";
    return false;
  }
  uint32_t flag_word = 0;
  memcpy(&flag_word, tag.payload, 4);
  cls = uint8_t(flag_word & 0xFFu);
  flags = flag_word & 0xFF00u;
  skip_element(c, tag);

  if (!read_tag(c, tag, error))
    return false;
  dims.resize(tag.size / 4);
  for (size_t i = 0; i < dims.size(); i++)
    memcpy(&dims[i], tag.payload + i * 4, 4);
  skip_element(c, tag);

  if (!read_tag(c, tag, error))
    return false;
  name.assign(reinterpret_cast<const char *>(tag.payload), tag.size);
  skip_element(c, tag);
  return true;
}

bool read_struct(cursor_t &c, const std::vector<int32_t> &dims, mat_value_t &out, std::string &error)
{
  tag_t tag;
  if (!read_tag(c, tag, error))
    return false;
  if (tag.size < 4)
  {
    error = "struct field-name-length subelement is too small";
    return false;
  }
  int32_t field_len = 0;
  memcpy(&field_len, tag.payload, 4);
  if (field_len <= 0)
  {
    error = "struct field name length is not positive";
    return false;
  }
  skip_element(c, tag);

  if (!read_tag(c, tag, error))
    return false;
  std::vector<std::string> field_names;
  for (uint32_t off = 0; off + uint32_t(field_len) <= tag.size; off += uint32_t(field_len))
  {
    const char *p = reinterpret_cast<const char *>(tag.payload + off);
    field_names.emplace_back(p, strnlen(p, size_t(field_len)));
  }
  skip_element(c, tag);

  size_t element_count = dims.empty() ? 0 : 1;
  for (auto d : dims)
    element_count *= size_t(d);

  out.kind = mat_value_t::kind_t::structure;
  out.struct_dims = dims;
  out.elements.resize(element_count);
  // ELEMENT-MAJOR: all of element 0's fields, then all of element 1's. A (1,5) camera struct is
  // therefore five consecutive runs of {points_index, xyz, pixels}, not three runs of five.
  for (size_t e = 0; e < element_count; e++)
  {
    for (const auto &field : field_names)
    {
      tag_t field_tag;
      if (!read_tag(c, field_tag, error))
        return false;
      if (field_tag.type != mi_matrix)
      {
        error = fmt::format("struct field '{}' is data type {}, expected a matrix", field, field_tag.type);
        return false;
      }
      cursor_t inner{c.begin, field_tag.payload + field_tag.size, field_tag.payload};
      auto value = std::make_shared<mat_value_t>();
      if (!read_matrix(inner, *value, error))
        return false;
      out.elements[e].fields.emplace_back(field, std::move(value));
      skip_element(c, field_tag);
    }
  }
  return true;
}

bool read_matrix(cursor_t &c, mat_value_t &out, std::string &error)
{
  uint8_t cls = 0;
  uint32_t flags = 0;
  std::vector<int32_t> dims;
  if (!read_matrix_prologue(c, cls, flags, dims, out.name, error))
    return false;

  if (flags & k_flag_complex)
  {
    error = "complex arrays are not supported";
    return false;
  }

  if (cls == mx_struct)
    return read_struct(c, dims, out, error);

  if (cls == mx_cell || cls == mx_object || cls == mx_sparse)
  {
    // Named rather than silently skipped: a caller asking for a field that turned out to be a cell
    // array should be told what it is, not handed an empty value.
    out.kind = mat_value_t::kind_t::unsupported;
    return true;
  }

  tag_t tag;
  if (!read_tag(c, tag, error))
    return false;

  if (cls == mx_char)
  {
    out.kind = mat_value_t::kind_t::text;
    if (tag.type == mi_uint16 || tag.type == mi_int16)
    {
      // UTF-16 in practice holds ASCII here; take the low byte rather than pull in a converter.
      out.text.reserve(tag.size / 2);
      for (uint32_t i = 0; i + 1 < tag.size; i += 2)
        out.text.push_back(char(tag.payload[i]));
    }
    else
    {
      out.text.assign(reinterpret_cast<const char *>(tag.payload), tag.size);
    }
    skip_element(c, tag);
    return true;
  }

  size_t expected = dims.empty() ? 0 : 1;
  for (auto d : dims)
    expected *= size_t(d);
  out.kind = mat_value_t::kind_t::numeric;
  out.numeric.dims = dims;
  if (!read_numeric_payload(tag, expected, out.numeric.data, error))
    return false;
  skip_element(c, tag);
  return true;
}

} // namespace

const mat_value_t *mat_struct_t::find(std::string_view name) const
{
  for (const auto &[field, value] : fields)
    if (field == name)
      return value.get();
  return nullptr;
}

const mat_value_t *mat_file_t::find(std::string_view name) const
{
  for (const auto &variable : variables)
    if (variable.name == name)
      return &variable;
  return nullptr;
}

bool inflate_zlib(const uint8_t *data, size_t size, size_t size_hint, std::vector<uint8_t> &out, std::string &error)
{
  // Grow-and-retry, because a deflate stream does not record its output size. The loop looks like it
  // could run away; in practice it runs once. Ford scans are mostly IEEE doubles and inflate to
  // 1.08-1.10x their compressed size, so a hint of 2x compressed clears every scan in the dataset on
  // the first pass. The doubling is there for correctness, not for the common case.
  size_t capacity = size_hint ? size_hint : size * 2 + 1024;
  for (int attempt = 0; attempt < 24; attempt++)
  {
    out.resize(capacity);
    zng_stream stream{};
    if (zng_inflateInit(&stream) != Z_OK)
    {
      error = "zng_inflateInit failed";
      return false;
    }
    stream.next_in = data;
    stream.avail_in = uint32_t(size);
    stream.next_out = out.data();
    stream.avail_out = uint32_t(out.size());
    // One shot: the whole input is in memory and so is the whole output, so there is no reason to
    // loop over chunks. Z_STREAM_END means it fit; Z_BUF_ERROR (or Z_OK with input left) means the
    // output buffer was too small and we go round again.
    const int rc = zng_inflate(&stream, Z_FINISH);
    const size_t produced = size_t(stream.total_out);
    // WHY avail_out DECIDES, and not the return code. Z_BUF_ERROR means only "no progress was
    // possible", which under Z_FINISH covers two completely different situations: the output buffer
    // filled up, or the input ran out mid-stream. Retrying is right for the first and a disaster for
    // the second -- a truncated file can never be satisfied, so the buffer just doubles until the
    // allocator gives up. One damaged scan in the Ford release (Scan0111.mat) hangs an import that
    // way, and it presents as the converter waiting forever on an input that never reports.
    //
    // If output room is LEFT OVER and the stream still did not end, the input is what ran out.
    const bool output_was_full = stream.avail_out == 0;
    const char *message = stream.msg ? stream.msg : "no message";
    zng_inflateEnd(&stream);
    if (rc == Z_STREAM_END)
    {
      out.resize(produced);
      return true;
    }
    if (!output_was_full)
    {
      error = fmt::format("truncated or corrupt zlib stream: inflate returned {} ({}) after {} bytes with {} bytes of output room to spare", rc, message, produced, capacity - produced);
      return false;
    }
    if (rc != Z_OK && rc != Z_BUF_ERROR)
    {
      error = fmt::format("inflate failed: {} ({})", rc, message);
      return false;
    }
    capacity *= 2;
  }
  error = fmt::format("inflate did not converge: output exceeded {} bytes", capacity);
  return false;
}

bool mat_read_file(const std::string &path, mat_file_t &out, std::string &error)
{
  std::vector<uint8_t> bytes;
  {
    FILE *f = fopen(path.c_str(), "rb");
    if (!f)
    {
      error = fmt::format("cannot open {}", path);
      return false;
    }
    fseek(f, 0, SEEK_END);
    const long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len < long(k_header_size))
    {
      fclose(f);
      error = fmt::format("{} is too small to be a MAT-file", path);
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
  }

  out.header_text.assign(reinterpret_cast<const char *>(bytes.data()), 116);
  while (!out.header_text.empty() && (out.header_text.back() == ' ' || out.header_text.back() == '\0'))
    out.header_text.pop_back();
  if (out.header_text.rfind("MATLAB 5.0", 0) != 0)
  {
    // v7.3 files are HDF5 and start with the HDF5 signature instead; say so, because "not a
    // MAT-file" would be actively misleading for one.
    error = fmt::format("{} is not a MATLAB 5.0 MAT-file (v7.3/HDF5 is not supported): '{}'", path, out.header_text.substr(0, 32));
    return false;
  }
  uint16_t endian = 0;
  memcpy(&endian, bytes.data() + 126, 2);
  if (endian != 0x4D49) // 'IM' little-endian; 'MI' would mean the file needs byte swapping
  {
    error = fmt::format("{} is big-endian, which this reader does not swap", path);
    return false;
  }

  // Decompressed payloads must outlive the values that point into them... except they do not: every
  // numeric read COPIES into mat_array_t::data, so each buffer can die at the end of its iteration.
  std::vector<uint8_t> inflated;
  cursor_t c{bytes.data(), bytes.data() + bytes.size(), bytes.data() + k_header_size};
  while (c.remaining() >= 8)
  {
    tag_t tag;
    if (!read_tag(c, tag, error))
      return false;
    if (tag.type == mi_compressed)
    {
      if (!inflate_zlib(tag.payload, tag.size, tag.size * 2 + 1024, inflated, error))
        return false;
      cursor_t inner{inflated.data(), inflated.data() + inflated.size(), inflated.data()};
      tag_t inner_tag;
      if (!read_tag(inner, inner_tag, error))
        return false;
      if (inner_tag.type != mi_matrix)
      {
        error = fmt::format("compressed element wraps data type {}, expected a matrix", inner_tag.type);
        return false;
      }
      cursor_t body{inflated.data(), inner_tag.payload + inner_tag.size, inner_tag.payload};
      mat_value_t value;
      if (!read_matrix(body, value, error))
        return false;
      out.variables.push_back(std::move(value));
    }
    else if (tag.type == mi_matrix)
    {
      cursor_t body{bytes.data(), tag.payload + tag.size, tag.payload};
      mat_value_t value;
      if (!read_matrix(body, value, error))
        return false;
      out.variables.push_back(std::move(value));
    }
    skip_element(c, tag);
  }
  if (out.variables.empty())
  {
    error = fmt::format("{} contains no variables", path);
    return false;
  }
  return true;
}

} // namespace ford
