# The Ford Campus Vision and Lidar dataset importer, twice: once in C++, once in Python.
#
# The point of having both is that they are the SAME program. Neither does per-point work in the
# host language -- both hand whole arrays across the public API and let the library do the pass.
# If the Python one needs something the C++ one did not, that is a hole in the bindings; if it needs
# something neither can express, that is a hole in the API.
#
# Only the C++ half builds. examples/ford/python is scripts against the `dew` wheel and is not a
# CMake target.
add_subdirectory(cpp)
