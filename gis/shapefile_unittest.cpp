#include "shapefile.hpp"

#define BOOST_TEST_MODULE shapefile_unittest
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_CASE( increment_iterator ) {
  shapefile_source in("path");
  shp::view in_view = in();
  //shapefile_source in = open_shapefile_source(shp, shx, dbf);
  for (auto feature = in_view.begin(); feature != in_view.end(); ++feature) {
  	feature.geometry.type();
  }
}



BOOST_AUTO_TEST_CASE( increment_iterator ) {
  io::mapped_file_source file("path");
  csv_source in(file);
  for (auto row = in.begin(); row != in.end(); ++row) {
    
  }
}
