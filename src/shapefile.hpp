#pragma once

#include <boost/endian/arithmetic.hpp>

template <class It>
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
};


struct shp_header {
  big_int32_t     file_code;
  big_int32_t			unused_1;
  big_int32_t			unused_2;
  big_int32_t			unused_3;
  big_int32_t			unused_4;
  big_int32_t			unused_5;
  big_int32_t     file_length;
  little_int32_t  version;
  little_int32_t  shape_type;
};

enum shape_type {
	null_shape = 0,
	point = 1,
	polyline = 3,
	polygone = 5,
	multipoint = 8,
	point_z = 11,
	polyline_z = 13,
	polygone_z = 15
	multipoint_z = 18,
	point_m = 21,
	polyline_m = 23,
	polygone_m = 25
	multipoint_m = 28,
	multipatch = 31,
};

struct point {
	double x;
	double y;
};

class any_feature {
 public:

 	template <class Feature> 
 	Feature as() const {
 		return Feature(*this);
 	}

 	operator point() const;

};

template <class Feature>
class features_iterator {

	std::pair<Feature, size_t> dereference() const;
}

template <class Feature>
class features_view {

};

class shp_source {
	source(filename)

	shape();
	size();
	dbf_source attributes();

	template <class Feature>
	features_view read();

	template <class Feature>
	void write(View);
	void write(type, View);

};

class shp_sink {
	sink(filename)

	template <class Feature>
	void write(View);
	void write(type, View);

};