cc_library(
  name = "io",
  hdrs = [
    "csv.hpp",
    "dbase.hpp",
    "iterator.hpp",
    "meta.hpp",
    "utility.hpp",
    "tuple.hpp",
    "shapefile.hpp",
    "mapped_file.hpp"
  ],
  deps = [
    "@boost//:iostreams",
  ],
)

cc_binary(
  name = "main",
  srcs = ["main.cpp"],
  deps = [
    ":io",
    "@geometry//:main"
  ],
)

cc_test(
  name = "csv_unittest",
  srcs = ["csv_unittest.cpp"],
  deps = [
    ":io",
  ],
)