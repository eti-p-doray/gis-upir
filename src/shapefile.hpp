#pragma once

#include <boost/endian/arithmetic.hpp>

namespace io {

/*template <class It>
class dbf_iterator {

};

class dbf_view {

};

class dbf_source {
	using fields_view = const std::vector<std::string>&;

	source(filename);

	fields_view fields();
	size();
	view read();
};

class dbf_sink {
	sink(filename);

	void write(fields, View);
};*/


/*struct shp_header {
  big_int32_t     file_code;
  big_int32_t			unused_1;
  big_int32_t			unused_2;
  big_int32_t			unused_3;
  big_int32_t			unused_4;
  big_int32_t			unused_5;
  big_int32_t     file_length;
  little_int32_t  version;
  little_int32_t  shape_type;
};*/

enum class shape_type {
	null_shape = 0,
	point = 1,
	polyline = 3,
	polygone = 5,
	multipoint = 8,
	point_z = 11,
	polyline_z = 13,
	polygone_z = 15,
	multipoint_z = 18,
	point_m = 21,
	polyline_m = 23,
	polygone_m = 25,
	multipoint_m = 28,
	multipatch = 31,
};

struct point {
	double x;
	double y;
};

class any_geometry {
 public:
 	/*operator point() const {
 		if (type_ != shape_type::point) {
 			//throw {};
 		}
 	}*/

 private:
 	shape_type type_;
};

template <class Geometry>
struct feature {
	Geometry geometry;
	//properties;
};

template <class Geometry>
class features_iterator {

	feature<Geometry> dereference() const;
};

class shp_source {
 public:
 	shp_source(const std::string& path)
 			: main_(path + ".shp"),
 				index_(path + ".shx"),
 				dbase_(path + ".dbf") {}

 	template <class Geometry>
 	class features_view {

 	};

	shape_type shape();
	size_t size();
	//dbf_source attributes();

	template <class Geometry = any_geometry>
	features_view<Geometry> read() const;

private:
  boost::iostreams::mapped_file_source main_;
  boost::iostreams::mapped_file_source index_;
  boost::iostreams::mapped_file_source dbase_;
};

/*class shp_sink {
	shp_sink(filename)

	template <class Feature>
	void write(View);
	void write(type, View);

};*/

}