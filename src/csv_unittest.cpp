#include "csv.hpp"

#define BOOST_TEST_MODULE csv_unittest
#include <boost/test/included/unit_test.hpp>

std::string csv_data = 
"1, 1.0, 2013-06-12 21:37:14\n"
"2, 2.0, 2013-06-12 21:37:21\n";

BOOST_AUTO_TEST_CASE( increment_iterator ) {
  auto it = io::make_csv_iterator(csv_data.begin(), csv_data.end());
  BOOST_CHECK(it != io::default_sentinel());
  ++it;
  BOOST_CHECK(it != io::default_sentinel());
  ++it;
  BOOST_CHECK(it == io::default_sentinel());
}

BOOST_AUTO_TEST_CASE( increment_row ) {
  auto it = io::make_csv_iterator(csv_data.begin(), csv_data.end());
  auto row = it->begin();
  BOOST_CHECK(row++ != it->end());
  BOOST_CHECK(row++ != it->end());
  BOOST_CHECK(row++ != it->end());
  BOOST_CHECK(row == it->end());
  row = it->begin();
  BOOST_CHECK(row++ != it->end());
  BOOST_CHECK(row++ != it->end());
  BOOST_CHECK(row++ != it->end());
  BOOST_CHECK(row == it->end());
}

BOOST_AUTO_TEST_CASE( dereference_row ) {
  auto it = io::make_csv_iterator(csv_data.begin(), csv_data.end());
  auto row = it->begin();
  BOOST_CHECK(int(*row++) == 1);
  BOOST_CHECK(double(*row++) == 1.0);
  BOOST_CHECK(io::csv_date(*row++).sec == 14);
  BOOST_CHECK(row == it->end());
  ++it;
  row = it->begin();
  BOOST_CHECK(int(*row++) == 2);
  BOOST_CHECK(double(*row++) == 2.0);
  BOOST_CHECK(io::csv_date(*row++).sec == 21);
  BOOST_CHECK(row == it->end());
}