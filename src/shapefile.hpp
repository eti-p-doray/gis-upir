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

enum class shape_type : int32_t {
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

struct multipoint_header {
	double box[4];
	uint32_t num_points;
};

struct polyline_header {
	double box[4];
	int32_t num_parts;
	int32_t num_points;
};

struct multipoint {
	double box[4];
	int32_t num_points;
	const point* points;
};

struct polyline {
	double box[4];
	int32_t num_parts;
	int32_t num_points;
	const int32_t* parts;
	const point* points;
};

struct record_header {
	int32_t number;
	int32_t length;
};

struct index_record {
	int32_t offset;
	int32_t length;
};

template <class It>
class geometry_reference {
 public:
 	geometry_reference(const char* data)
 			: type_(*as<shape_type>(data)), 
 				data_(data + sizeof(int32_t)) {}

 	operator point() const {
 		check_shape(shape_type::point);
 		return *as<point>(data_);
 	}
 	operator multipoint() const {
 		check_shape(shape_type::multipoint);
 		multipoint v = *as<multipoint>(data_);
 		v.points = as<point>(data_ + sizeof(multipoint_header));
 		return v;
 	}
 	operator polyline() const {
 		check_shape(shape_type::polyline);
 		multipoint v = *as<polyline>(data_);
 		v.parts = as<int32_t>(data_ + sizeof(polyline_header));
 		v.points = as<point>(data_ + sizeof(multipoint_header) + 
 												 v.num_parts * sizeof(int32_t));
 		return v;
 	}

 private:
 	void check_shape(shape_type type) const {
 		if (type_ != type) {
 			throw std::bad_cast();
 		}
 	}

 	shape_type type_;
 	const char* data_;
};

struct feature {
	geometry_reference geometry;
	//properties;
};

class features_iterator {
 private:
	feature<Geometry> dereference() const {
		return {record_ + sizeof(record_header)};
	}
	void increment() {
		int32_t length = as<record_header>(record_)->length;
		record_ += length;
	}
	bool equal(const features_iterator& that) const {
		return record_ == that.record_;
	}
	void decrement() { advance(-1);	}
	void advance(std::ptrdiff_t n) {
		int32_t old_idx = index();
		int32_t new_idx = old_idx + n;
		record_ = first_ + 
				(as<index_record>(index_ + new_idx * sizeof(index_record))->offset -
				 as<index_record>(index_ + old_idx * sizeof(index_record))->offset);
	}
	void distance_to(const features_iterator& that) {
		return that.index() - index();
	}

	int32_t index() const {
		return as<record_header>(record_)->number;
	}

	const char* record_;
	const char* index_;
	const char* properties_;
};

namespace detail {

struct shp_header {
	big_int32_t file_code;
  big_int32_t			unused_1;
  big_int32_t			unused_2;
  big_int32_t			unused_3;
  big_int32_t			unused_4;
  big_int32_t			unused_5;
  big_int32_t     file_length;
  little_int32_t  version;
  little_int32_t  shape_type;
  double 					x_min;
  double 					y_min;
  double 					x_max;
  double 					y_max;
  double 					z_min;
  double 					z_max;
  double 					m_min;
  double 					m_max;
};

struct dbf_header {
	struct date {
		uint8_t year;
		uint8_t month;
		uint8_t day;
	};

	uint8_t identifier;
	date last_updated;
	uint32_t num_records;
	uint16_t header_length;
	uint32_t record_length;
	uint16_t reserved_1;
	uint8_t incomplete_flag;
	uint8_t encryption_flag;
	uint8_t reserved_dos[12];
	uint8_t production_mdx_flag;
	uint8_t language_driver_id;
	uint8_t reserved_2[2];
};

struct dbf_field_descriptor {
	const char name[11];
	uint8_t type;
	uint8_t reserved_1[4];
	uint8_t length;
	uint8_t decimal_count;
	uint16_t work_area_id;
	uint16_t example;
	uint8_t reserved_2[10];
	uint8_t production_mdx_flag;
};

}

struct dbase_date {
	int year;
	int month;
	int day;
};

class dbase_row {
	class reference {
	 public:
	 	operator std::string() const {
	 		return {first, last_};
	 	}
	 	operator int() const { return extract<int, qi::int_type>(); }
    operator double() const { return extract<double, qi::double_type>(); }
    operator dbase_date() const { return extract<csv_date, csv_date_parser<It>>(); }
    operator std::string() const { return std::string(first_, last_); }


	 	template <class T, class P>
    T extract() const {
      using boost::phoenix::ref; using qi::_1;
      T v = {};
      P parser;
      qi::phrase_parse(first_, last_, parser[ref(v) = _1], qi::space);
      return v;
    }

	 private:
	 	const char* first;
	 	const char* last_;
	};

	class iterator {

	};
};

class dbase_iterator {

};

class dbase_view {



private:
	void parse_header() {

	}

	const char* first_;
};

class features_view {
 public:
	using iterator = features_iterator;

	features_iterator(...) {
		parse_shp(main_);
		parse_shp(index_);
		parse_dbf();
	}

	iterator begin() const;
	iterator end() const;

 private:
 	static shape_type parse_shapefile_header(const char* data) {
		auto first = data;
		const detail::shp_header* header = ptr<detail::shp_header>(first);
		if (header == nullptr) {
			return false;
		}
		if (header->file_code != 9994) {
			return false;
		}
		return header->shape_type;
	}

 	const char* main_;
 	const char* index_;
 	dbase_view dbase_;
};

class shapefile_source {
 public:
 	shapefile_source(const std::string& path)
 			: main_(path + ".shp"),
 				index_(path + ".shx"),
 				dbase_(path + ".dbf") {

 		parse_shp();
 	}

	shape_type shape();
	size_t size();
	//dbf_source attributes();

	features_view<const char*> read() const;

private:
  boost::iostreams::mapped_file_source main_;
  boost::iostreams::mapped_file_source index_;
  boost::iostreams::mapped_file_source dbase_;

  size_t size_;
  shape_type shape_;
};

/*class shp_sink {
	shp_sink(filename)

	template <class Feature>
	void write(View);
	void write(type, View);

};*/

}