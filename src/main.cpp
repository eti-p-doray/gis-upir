#include <stdio.h>

#include "mapped_file.hpp"
#include "csv.hpp"
#include "shapefile.hpp"

int main(int argc, char **argv) {
  io::csv_source in("bike_path/Chunk_1_mm.csv");
  auto in_view = in.read(io::skip_header);
  for (auto row = in_view.begin(); row != in_view.end(); ++row) {
    for (auto cell = row->begin(); cell != row->end(); ++cell) {
      int a = *cell;
      double b = *cell;
      io::csv_date c = *cell;
    }

    auto t = extract<ignore_t, int, io::csv_date>(*row);
  }
  std::cout << "done" << std::endl;
}