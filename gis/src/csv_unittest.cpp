#include "csv.hpp"

#define BOOST_TEST_MODULE csv_unittest
#include <boost/test/included/unit_test.hpp>

std::string csv_data = 
"1, 1.0, 2013-06-12 21:37:14\n"
"2, 2.0, 2013-06-12 21:37:21\n";

BOOST_AUTO_TEST_CASE( increment_row ) {
  auto view = io::make_csv_view(csv_data);
  auto row = view.begin();
  BOOST_CHECK(row != view.end());
  ++row;
  BOOST_CHECK(row != view.end());
  ++row;
  BOOST_CHECK(row == view.end());
}

BOOST_AUTO_TEST_CASE( increment_cell ) {
  auto view = io::make_csv_view(csv_data);
  auto row = view.begin();
  auto cell = row->begin();
  BOOST_CHECK(cell++ != row->end());
  BOOST_CHECK(cell++ != row->end());
  BOOST_CHECK(cell++ != row->end());
  BOOST_CHECK(cell == row->end());
  cell = row->begin();
  BOOST_CHECK(cell++ != row->end());
  BOOST_CHECK(cell++ != row->end());
  BOOST_CHECK(cell++ != row->end());
  BOOST_CHECK(cell == row->end());
}

BOOST_AUTO_TEST_CASE( dereference_cell ) {
  auto view = io::make_csv_view(csv_data);
  auto row = view.begin();
  auto cell = row->begin();
  BOOST_CHECK(int(*cell++) == 1);
  BOOST_CHECK(double(*cell++) == 1.0);
  BOOST_CHECK(io::csv::date(*cell++).sec == 14);
  BOOST_CHECK(cell == row->end());
  ++row;
  cell = row->begin();
  BOOST_CHECK(int(*cell++) == 2);
  BOOST_CHECK(double(*cell++) == 2.0);
  BOOST_CHECK(io::csv::date(*cell++).sec == 21);
  BOOST_CHECK(cell == row->end());
}