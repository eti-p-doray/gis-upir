#include <stdio.h>

#include "mapped_file.hpp"
#include "csv.hpp"

int main(int argc, char **argv) {
  csv_source in("bike_path/Chunk_1_mm.csv");
  auto in_view = in.read(skip_header);
  for (auto it = in_view.begin(); it != in_view.end(); ++it) {
    auto v = it->extract<ignore_t, date, double>();
  }
  std::cout << "done" << std::endl;
}