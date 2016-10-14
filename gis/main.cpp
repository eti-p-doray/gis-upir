#include <stdio.h>

#include "shapefile.hpp"
#include "mapped_file.hpp"
#include "csv.hpp"
#include "shapefile.hpp"
#include "tuple.hpp"

int main(int argc, char **argv) {
  io::shapefile_source in(
    "intersections/intersections_on_bike_network_with_change_in_road_type");
  auto in_view = in();
  

  for (auto&& record : in_view) {
    io::shp::point p = record.geometry;
    std::cout << p.x << " " << p.y << std::endl;
  }
}
