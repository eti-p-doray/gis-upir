#pragma once

#include <boost/endian/arithmetic.hpp>
#include <boost/iterator/iterator_facade.hpp>

#include "utility.hpp"
#include "iterator.hpp"
#include "dbase.hpp"
#include "mapped_file.hpp"

namespace io {

namespace shp {

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

#pragma pack(push, 1)
struct point {
	double x;
	double y;
};
static_assert(sizeof(point) == 16, 
  "shp::point is not packed");

struct multipoint_header {
	double box[4];
	uint32_t num_points;
};
static_assert(sizeof(multipoint_header) == 36, 
  "shp::multipoint_header is not packed");

struct multipoint {
	double box[4];
	int32_t num_points;
	const point* points;
};

struct polyline_header {
	double box[4];
	int32_t num_parts;
	int32_t num_points;
};
static_assert(sizeof(polyline_header) == 40, 
  "shp::polyline_header is not packed");

struct polyline {
	double box[4];
	int32_t num_parts;
	int32_t num_points;
	const int32_t* parts;
	const point* points;
};
#pragma pack(pop)

namespace detail {

using namespace boost::endian;
#pragma pack(push, 1)

struct header {
	big_int32_t 		file_code;
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
static_assert(sizeof(header) == 100, 
  "shp::header is not packed");

struct record_header {
	big_int32_t number;
	big_int32_t length;
};
static_assert(sizeof(record_header) == 8, 
  "shp::record_header is not packed");

struct index_record {
	big_int32_t offset;
	big_int32_t length;
};
static_assert(sizeof(index_record) == 8, 
  "shp::index_record is not packed");

#pragma pack(pop)
} // namespace detail

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
 		polyline v = *as<polyline>(data_);
 		v.parts = as<int32_t>(data_ + sizeof(polyline_header));
 		v.points = as<point>(data_ + sizeof(polyline_header) + 
 												 v.num_parts * sizeof(int32_t));
 		return v;
 	}

 	point as_point() const { return point(*this); }
 	multipoint as_multipoint() const { return multipoint(*this); }
 	polyline as_polyline() const { return polyline(*this); }

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
	dbf::record properties;
};

class features_iterator :
    public boost::iterator_facade<features_iterator, feature,
                                  std::random_access_iterator_tag,
                                  feature> {
 public:
 	features_iterator(const char* record, 
 										const detail::index_record* index,
 										dbf::records_iterator properties)
 			: record_(record),
 				index_(index),
 				properties_(properties) {}

 private:
 	friend class boost::iterator_core_access;

	feature dereference() const {
		return {
			geometry_reference(record_ + sizeof(detail::record_header)), 
			*properties_
		};
	}
	void increment() {
		int32_t length = as<detail::record_header>(record_)->length * sizeof(uint16_t);
		record_ += length + sizeof(detail::record_header);
		++properties_;
	}
	bool equal(const features_iterator& that) const {
		return record_ == that.record_;
	}
	void decrement() { advance(-1);	}
	void advance(std::ptrdiff_t n) {
		int32_t old_idx = index();
		int32_t new_idx = old_idx + n;
		record_ += (index_[new_idx].offset - index_[old_idx].offset) * sizeof(uint16_t);
		properties_ += n;
	}
	std::ptrdiff_t distance_to(const features_iterator& that) const {
		return that.index() - index();
	}

	int32_t index() const {
		return as<detail::record_header>(record_)->number;
	}

	const char* record_;
	const detail::index_record* index_;
	dbf::records_iterator properties_;
};

class features_view {
 public:
	features_view(const char* main_first, const char* main_last,
								const char* index_first, const char* index_last,
								const char* dbase_first, const char* dbase_last) 
			: main_first_(main_first), main_last_(main_last),
				index_first_(index_first), index_last_(index_last),
				dbase_(dbase_first, dbase_last) {
		header_ = parse_header(main_first_, main_last_);
		parse_header(index_first_, index_last_);
	}

	shape_type shape() const {
		return static_cast<shape_type>(uint32_t(header_->shape_type));
	}
	size_t size();
	//dbf_source attributes();

	features_iterator begin() const { 
		return {
			main_first_ + sizeof(detail::header),
			as<detail::index_record>(index_first_ + sizeof(detail::header)),
			dbase_.begin()
		};
	}
	features_iterator end() const { 
		return {
			main_last_,
			as<detail::index_record>(index_first_ + sizeof(detail::header)),
			dbase_.end()
		};
	}

 private:
 	static const detail::header* 
 			parse_header(const char* first, const char* last) {
		const detail::header* header = as<detail::header>(first, last);
		if (header == nullptr) {
			return nullptr;
		}
		if (header->file_code != 9994) {
			return nullptr;
		}
		return header;//header->shape_type;
	}

 	const char* main_first_;
 	const char* main_last_;
 	const char* index_first_;
 	const char* index_last_;
 	const detail::header* header_;
 	dbf::view dbase_;
};


} // namespace shp

template <class Rng1, class Rng2, class Rng3>
shp::features_view 
		make_shapefile_view(const Rng1& main, 
												const Rng2& index, 
												const Rng3& dbase) {
  using std::begin; using std::end;
  return {
  	begin(main), end(main),
  	begin(index), end(index),
  	begin(dbase), end(dbase)	
  };
}

class shapefile_source {
 public:
 	shapefile_source(const std::string& path)
 			: main_(path + ".shp"),
 				index_(path + ".shx"),
 				dbase_(path + ".dbf") {}

	shp::features_view operator()() const {
		return make_shapefile_view(main_, index_, dbase_);
	}

private:
  boost::iostreams::mapped_file_source main_;
  boost::iostreams::mapped_file_source index_;
  boost::iostreams::mapped_file_source dbase_;
};

} // namespace io