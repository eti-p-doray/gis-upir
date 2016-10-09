#include <stdio.h>

#include "mapped_file.hpp"
#include "csv.hpp"

int main(int argc, char **argv) {
  mapped_file in("bike_path/Chunk_1_mm.csv");
  for (auto it = make_csv_iterator(in.begin(), in.end(), skip_header);
      it != default_sentinel(); ++it) {
    auto v = it->extract<ignore_t, date, double>();
  }
  std::cout << "done" << std::endl;
}