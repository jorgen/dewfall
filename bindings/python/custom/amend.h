/* Hand-written binding for the attribute AMEND path -- joining a new attribute onto an
 * already-converted dataset by key.
 *
 * dew_converter_add_data_for_attribute takes `const uint64_t *keys` and a bare `const void *values`
 * with a single `count`. The generator can be told those are count-length buffers (//= arrays:),
 * which is enough to type them for the C++ binding, but not enough to make a SAFE Python call: the
 * converter reads count * element_size(type) * components bytes out of `values` using the type the
 * attribute was DECLARED with, and nothing in the call itself says what that is. A caller who
 * declares r64 and passes a float32 array gets a heap over-read of exactly half the buffer.
 *
 * So this layer remembers what add_attribute declared and checks the array against it, in the same
 * spirit as custom/file_convert_callbacks.h validating what the reader would otherwise trust. The
 * check is on TOTAL BYTES rather than on the dtype, which catches a wrong element type and a wrong
 * component count with one comparison and keeps numpy's dtype zoo out of the binding.
 *
 * The keys/values parameter types carry nb::c_contig, so nanobind rejects a sliced or transposed
 * view at the cast rather than letting the converter read it as if it were packed.
 *
 * Included by the GENERATED dew_bindings_generated.cpp after file_convert_callbacks.h, whose
 * element_size_for() it reuses.
 */
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>

#include <string>
#include <unordered_map>

namespace dewpy
{

/* What add_attribute declared, per converter, per attribute name.
 *
 * Keyed on the raw converter handle because that is the only identity the generated holder exposes.
 * Entries are a few dozen bytes and live until the process ends -- there is no destroy hook to hang
 * cleanup off, and a converter declares a handful of attributes at most. */
struct amend_schema_t
{
  enum dew_type_t type = dew_type_u8;
  enum dew_components_t components = dew_components_1;
};

inline std::unordered_map<const void *, std::unordered_map<std::string, amend_schema_t>> &amend_schemas()
{
  static std::unordered_map<const void *, std::unordered_map<std::string, amend_schema_t>> schemas;
  return schemas;
}

/* Bytes the converter will read per value for this declared attribute. Mirrors the library's own
 * stride formula: size_for_format(type) * (int)components. */
inline uint64_t amend_value_stride(const amend_schema_t &schema)
{
  return uint64_t(element_size_for(schema.type)) * uint64_t(int(schema.components));
}

template <class ClsT> void bind_amend(ClsT &cls)
{
  using Holder = typename ClsT::Type;

  /* Replaces the generated add_attribute (see _CUSTOM_CLASS_SNIPPETS) purely to record the schema;
   * the call it forwards is identical. */
  cls.def(
    "add_attribute",
    [](Holder &self, std::string_view name, std::string_view key_attribute, enum dew_type_t type, enum dew_components_t components) {
      const uint8_t ok = dew_converter_add_attribute(self.h, name.data(), uint32_t(name.size()), key_attribute.data(), uint32_t(key_attribute.size()), type, components);
      if (ok)
        amend_schemas()[self.h][std::string(name)] = amend_schema_t{type, components};
      return ok;
    },
    nb::arg("name"), nb::arg("key_attribute"), nb::arg("type"), nb::arg("components"),
    "Declare an attribute the dataset does not have yet, and the existing attribute to join it on.\n\n"
    "Values are buffered until commit_attributes(). Returns 0 and sets the converter to the error\n"
    "state if the dataset is not mutable, the name is already taken, or the key attribute is not a\n"
    "single-component integer type.");

  cls.def(
    "add_data_for_attribute",
    [](Holder &self, std::string_view name, nb::ndarray<uint64_t, nb::ndim<1>, nb::c_contig> keys, nb::ndarray<nb::c_contig> values) {
      const std::string key(name);
      auto converter_entry = amend_schemas().find(self.h);
      if (converter_entry == amend_schemas().end() || converter_entry->second.find(key) == converter_entry->second.end())
        throw nb::value_error("add_data_for_attribute: no such attribute declared on this converter -- call add_attribute first");
      const amend_schema_t &schema = converter_entry->second[key];

      const uint64_t count = keys.shape(0);
      if (values.ndim() < 1 || values.shape(0) != count)
        throw nb::value_error("add_data_for_attribute: values must have one row per key");

      /* One comparison covering both a wrong element type and a wrong component count. */
      const uint64_t expected = count * amend_value_stride(schema);
      const uint64_t actual = uint64_t(values.size()) * uint64_t(values.itemsize());
      if (actual != expected)
        throw nb::value_error("add_data_for_attribute: values is the wrong size for the declared attribute type");

      return dew_converter_add_data_for_attribute(self.h, name.data(), uint32_t(name.size()), static_cast<const uint64_t *>(keys.data()), values.data(), count);
    },
    nb::arg("name"), nb::arg("keys"), nb::arg("values"),
    "Add (key, value) pairs for a declared attribute. Repeatable; values are buffered until\n"
    "commit_attributes(). `keys` is uint64 regardless of the key attribute's own width, and\n"
    "`values` must be C-contiguous with one row per key and the dtype the attribute was declared\n"
    "with. The last value added for a key wins.");
}

} // namespace dewpy
