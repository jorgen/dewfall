/* A reader for the subset of MATLAB Level 5 (.mat) that the Ford dataset uses.
 *
 * Not a general MATLAB reader, and deliberately so. It handles numeric arrays, char arrays, struct
 * arrays and the single-deflate-stream wrapper -- which is everything a Ford scan contains -- and
 * refuses the rest (cell arrays, sparse, objects, complex, v7.3/HDF5) rather than half-supporting
 * it. The format is documented in MathWorks' "MAT-File Format" guide.
 *
 * THE TRAP, and it is a quiet one. A MATLAB array has both a CLASS (mxDOUBLE_CLASS, ...) and a
 * DATA TYPE for the bytes on disk (miINT32, miDOUBLE, ...). They routinely disagree: MATLAB stores
 * a double array in the narrowest integer type its values fit into, so `points_index`, a double
 * array of indices below 2^31, is written as miINT32. Reading it as f64 because the class says
 * double yields negative indices and NaNs that look like data corruption. Every numeric read here
 * therefore converts from the DATA type and ignores the class except to reject what it cannot do.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace ford
{

// A numeric array, always widened to double on read.
//
// Widening rather than preserving the on-disk type is the right trade for this reader: the class
// is what the producer meant (Ford writes doubles throughout), the on-disk type is a storage
// detail, and a caller that had to switch on the storage type would be re-implementing the trap
// above at every use site. The cost is memory, and a Ford scan's largest array is 3 x 77122.
struct mat_array_t
{
  std::vector<int32_t> dims;
  std::vector<double> data; // COLUMN-MAJOR, as MATLAB stores it

  [[nodiscard]] size_t count() const
  {
    size_t n = dims.empty() ? 0 : 1;
    for (auto d : dims)
      n *= size_t(d);
    return n;
  }
  [[nodiscard]] int32_t rows() const
  {
    return dims.size() > 0 ? dims[0] : 0;
  }
  [[nodiscard]] int32_t cols() const
  {
    return dims.size() > 1 ? dims[1] : 0;
  }
  // Element (r, c) of a 2-D array. Column-major: MATLAB's (3, N) point array stores each point's
  // three components contiguously, so at(0..2, i) is point i -- the layout a bulk copy wants.
  [[nodiscard]] double at(int32_t r, int32_t c) const
  {
    return data[size_t(c) * size_t(rows()) + size_t(r)];
  }
  [[nodiscard]] const double *column(int32_t c) const
  {
    return data.data() + size_t(c) * size_t(rows());
  }
  [[nodiscard]] bool empty() const
  {
    return data.empty();
  }
};

struct mat_value_t;

// One element of a struct array: field name -> value. Small and linear -- a Ford scan's structs
// have five fields, and a map would cost more in ceremony than it saves in lookups.
struct mat_struct_t
{
  std::vector<std::pair<std::string, std::shared_ptr<mat_value_t>>> fields;

  [[nodiscard]] const mat_value_t *find(std::string_view name) const;
};

struct mat_value_t
{
  enum class kind_t
  {
    numeric,
    text,
    structure,
    unsupported
  };

  kind_t kind = kind_t::unsupported;
  std::string name;                       // the variable's name; empty for struct fields
  mat_array_t numeric;                    // kind_t::numeric
  std::string text;                       // kind_t::text
  std::vector<int32_t> struct_dims;       // kind_t::structure -- a struct ARRAY has a shape
  std::vector<mat_struct_t> elements;     // kind_t::structure, struct_dims elements, column-major

  [[nodiscard]] const mat_value_t *field(std::string_view field_name) const
  {
    return elements.empty() ? nullptr : elements.front().find(field_name);
  }
};

struct mat_file_t
{
  std::string header_text; // the 116-byte descriptive header, trimmed
  std::vector<mat_value_t> variables;

  [[nodiscard]] const mat_value_t *find(std::string_view name) const;
};

// Read a whole .mat file. `error` is set and false returned on any failure; the file is read into
// memory in one go (a Ford scan is ~6.5 MB) because its payload is a single deflate stream anyway,
// so nothing is gained by streaming the container.
bool mat_read_file(const std::string &path, mat_file_t &out, std::string &error);

// Inflate an RFC1950 (zlib-wrapped) stream. Exposed because the size guess is the interesting part
// and worth testing on its own: a deflate stream does not record its output size, so the caller
// starts from `size_hint` and grows. Ford scans inflate to 1.08-1.10x their compressed size --
// mostly IEEE doubles, which barely compress -- so the default guess lands in one pass.
bool inflate_zlib(const uint8_t *data, size_t size, size_t size_hint, std::vector<uint8_t> &out, std::string &error);

} // namespace ford
