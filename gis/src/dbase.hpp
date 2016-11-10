#pragma once

#include <string>

#include <boost/endian/arithmetic.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/utility/string_ref.hpp>

#include "iterator.hpp"
#include "utility.hpp"
#include "mapped_file.hpp"

namespace io {

namespace dbf {
namespace detail {
#pragma pack(push, 1)

struct header {
	struct date {
		uint8_t year;
		uint8_t month;
		uint8_t day;
	};

	uint8_t identifier;
	date last_updated;
	uint32_t num_records;
	uint16_t header_length;
	uint16_t record_length;
	uint8_t reserved_1[2];
	uint8_t incomplete_flag;
	uint8_t encryption_flag;
	uint8_t reserved_dos[12];
	uint8_t production_mdx_flag;
	uint8_t language_driver_id;
	uint8_t reserved_2[2];
};
static_assert(sizeof(header) == 32, 
  "dbf::header is not packed");

struct field_descriptor {
	const char name[11];
	uint8_t type;
	uint8_t reserved_1[4];
	uint8_t length;
	uint8_t decimal_count;
	uint16_t work_area_id;
	uint8_t example;
	uint8_t reserved_2[10];
	uint8_t production_mdx_flag;
};
static_assert(sizeof(field_descriptor) == 32, 
  "dbf::field_descriptor is not packed");

#pragma pack(pop)
} // namespace detail

struct date {
	int year;
	int month;
	int day;
};

enum class field_type : char {
	character = 'C',
	date = 'D',
	floating_point = 'F',
	logical = 'L',
	memo = 'M',
	numeric = 'N',
};

struct field_descriptor {
	std::string name;
	field_type type;
};

class field_reference {
  using fields_ptr = const detail::field_descriptor*;
 public:
  field_reference(fields_ptr descriptor)
      : descriptor_(descriptor) {}

  field_type type() const {
    return static_cast<field_type>(descriptor_->type);
  }
  boost::string_ref name() const {
    return {descriptor_->name, size_t(10)}; 
  }
  uint8_t length() const { return descriptor_->length; }
  uint8_t decimal_count() const { return descriptor_->decimal_count; }

 private:
  const fields_ptr descriptor_;
};

class field_iterator :
  public boost::iterator_facade<field_iterator, field_reference,
                                std::random_access_iterator_tag,
                                field_reference> {
  using fields_ptr = const detail::field_descriptor*;
 public:
  field_iterator(fields_ptr descriptor)
      : descriptor_(descriptor) {}

 private:
  friend class boost::iterator_core_access;

  field_reference dereference() const {
    return {descriptor_};
  }
  void increment() {
    ++descriptor_;
  }
  bool equal(field_iterator that) const {
    return descriptor_ == that.descriptor_;
  }

  fields_ptr descriptor_;
};

class field_view {
  using fields_ptr = const detail::field_descriptor*;
 public:

  field_view(fields_ptr first, fields_ptr last)
      : first_(first), last_(last) {}

  field_iterator begin() const { return {first_}; }
  field_iterator end() const { return {last_}; }

 private:
  fields_ptr first_;
  fields_ptr last_;
};

class record {
 public:
	class reference {
	 public:
	 	reference(const detail::field_descriptor* descriptor,
              const char* first, const char* last)
        : field_(descriptor),
          first_(first), last_(last) {}

    field_reference field() const { return field_; }

    operator boost::string_ref() const {
      return {first_, size_t(first_ - last_)}; 
    }
	 	operator std::string() const { return {first_, last_}; }
	 	operator int() const { return extract<int>(*this); }
    operator double() const { return extract<double>(*this); }
    //operator dbase_date() const { return extract<csv_date, csv_date_parser<It>>(); }
    //operator std::string() const { return std::string(first_, last_); }

    const char* begin() const { return first_; }
    const char* end() const { return last_; }

	 private:
    field_reference field_;
	 	const char* first_;
	 	const char* last_;
	 	field_type type_;
	};

	class iterator :
      public boost::iterator_facade<iterator, reference,
                                  std::forward_iterator_tag,
                                  reference> {
   public:
    iterator(const detail::field_descriptor* descriptors, const char* data)
      : descriptor_(descriptors), current_(data) {}

   private:
    friend class boost::iterator_core_access;
    
    reference dereference() const {
      return {descriptor_, current_, current_ + descriptor_->length};
    }
    void increment() {
    	current_ += descriptor_->length;
    	++descriptor_;
    }
    bool equal(const iterator& that) const {
      return current_ == that.current_;
    }

    const detail::field_descriptor* descriptor_;
    const char* current_;
  };
	
  record() = default;
  record(const detail::field_descriptor* descriptors, 
  			const char* first, const char* last)
      : descriptors_(descriptors),
       	first_(first), last_(last) {}

  iterator begin() const { return {descriptors_, first_ + 1}; }
  iterator end() const { return {nullptr, last_}; }

 private:
  const detail::field_descriptor* descriptors_;
  const char* first_;
  const char* last_;
};

class records_iterator :
    public boost::iterator_facade<records_iterator, record,
                                  std::random_access_iterator_tag,
                                  record> {
 public:
  records_iterator() = default;
  records_iterator(uint16_t record_length,
  				 const detail::field_descriptor* descriptors, 
  				 const char* current) 
  		: record_length_(record_length),
  			descriptors_(descriptors),
  			current_(current) {}
 
 private:
  friend class boost::iterator_core_access;
  
  record dereference() const {
    return {descriptors_, current_, current_ + record_length_};
  }
  void increment() {
    current_ += record_length_;
  }
  bool equal(const records_iterator& that) const {
    return current_ == that.current_;
  }
  void decrement() { 
  	current_ -= record_length_;	
  }
	void advance(std::ptrdiff_t n) {
		current_ += n * record_length_;
	}
	std::ptrdiff_t distance_to(const records_iterator& that) const {
		return (that.current_ - current_);
	}
  
  uint16_t record_length_;
  const detail::field_descriptor* descriptors_;
  const char* current_ = nullptr;
};

class view {
 public:
  view(const char* first, const char* last) 
  		: first_(first), last_(last) {
  	header_ = as<detail::header>(first_, last_);
  }

  field_view fields() const {
    return {
      descriptors(),
      descriptors() + 
        (header_->header_length - sizeof(detail::header)) / 
          sizeof(detail::field_descriptor)
    };
  }

  records_iterator begin() const {
  	return {
      header_->record_length, 
  		descriptors(), 
  		first_ + header_->header_length
    };
  }
  records_iterator end() const {
  	return {
      header_->record_length, 
  		descriptors(), 
  		last_
    }; 
  }

 private:
  const detail::field_descriptor* descriptors() const {
    return as<detail::field_descriptor>(first_ + sizeof(detail::header));
  }

  const char* first_;
  const char* last_;
  const detail::header* header_;
};


} // namespace dbf

template <class Rng>
dbf::view make_dbase_view(const Rng& rng) {
  using std::begin; using std::end;
  return {begin(rng), end(rng)};
}

class dbase_source {
 public:
  dbase_source(const std::string& path)
      : file_(path) {}

  dbf::view operator()() const {
    return make_dbase_view(file_);
  }

private:
  boost::iostreams::mapped_file_source file_;
};

} // namespace io