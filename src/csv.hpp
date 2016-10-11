#pragma once

#include <boost/iterator/iterator_facade.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

#include "mapped_file.hpp"
#include "iterator.hpp"
#include "tuple.hpp"
#include "utility.hpp"

namespace io {

namespace qi = boost::spirit::qi;

struct csv_date {
  int year;
  int month;
  int day;
  int hour;
  int min;
  int sec;
};

template <typename Iterator>
struct csv_date_parser : qi::grammar<Iterator, csv_date()> {
  csv_date_parser() : csv_date_parser::base_type(start) {
    start %= qi::lexeme[qi::int_ >> '-' >> qi::int_ >> '-' >> qi::int_ >> ' ' >>
                        qi::int_ >> ':' >> qi::int_ >> ':' >> qi::int_];
  }
  qi::rule<Iterator, csv_date()> start;
};

template <class It>
class csv_row {
 public:
  class reference {
   public:
    reference(It first, It last)
        : first_(first), last_(last) {}

    operator int() const { return extract<double, qi::int_type>(); }
    operator double() const { return extract<double, qi::double_type>(); }
    operator csv_date() const { return extract<csv_date, csv_date_parser<It>>(); }
    operator std::string() const { return std::string(first_, last_); }

   private:
    template <class T, class P>
    T extract() const {
      using boost::phoenix::ref; using qi::_1;
      T v = {};
      P parser;
      qi::phrase_parse(first_, last_, parser[ref(v) = _1], qi::space);
      return v;
    }

    It first_;
    It last_;
  };

  class iterator :
      public boost::iterator_facade<iterator, reference,
                                  std::forward_iterator_tag,
                                  reference> {
   public:
    iterator(It first, It last)
      : current_(first), next_(first), last_(last) {
      increment();
    }

   private:
    friend class boost::iterator_core_access;
    friend class default_sentinel;
    
    reference dereference() const {
      return {current_, next_};
    }
    void increment() {
      current_ = next_;
      while (next_ != last_ && !iscomma(*next_) && !iseol(*next_)) {
        ++next_;
      } if (iscomma(*next_)) ++next_;
    }
    bool equal(default_sentinel that) const {
      return current_ == next_;
    }

    It current_;
    It next_;
    It last_;
  };

  csv_row() = default;
  csv_row(It first, It last)
      : first_(first), last_(last) {}

  iterator begin() const { return {first_, last_}; }
  default_sentinel end() const { return {}; }

 private:
  It first_ = nullptr;
  It last_ = nullptr;
};

template <class It>
class csv_iterator :
    public boost::iterator_facade<csv_iterator<It>, csv_row<It>,
                                  std::forward_iterator_tag,
                                  csv_row<It>> {
 public:
  csv_iterator() = default;
  csv_iterator(It first, It last)
      : current_(first), last_(last) {}
  csv_iterator(It first, It last, skip_header_t)
      : current_(first), last_(last) {
    increment();
  }
 
 private:
  friend class boost::iterator_core_access;
  friend class default_sentinel;
  
  csv_row<It> dereference() const {
    return {current_, last_};
  }
  void increment() {
    while (current_ != last_ && !iseol(*current_)) {
      ++current_;
    } if (current_ != last_) ++current_;
  }
  bool equal(default_sentinel that) const {
    return current_ == last_;
  }
  
  It current_ = nullptr;
  It last_ = nullptr;
};

template <class It>
csv_iterator<It> make_csv_iterator(It first, It last) {
  return {first, last};
}
template <class It>
csv_iterator<It> make_csv_iterator(It first, It last, skip_header_t) {
  return {first, last, skip_header};
}

class csv_source {
 public:

  class view {
   public:
    using iterator = csv_iterator<const char*>;

    view(iterator first) : first_(first) {}
    iterator begin() const { return first_; }
    default_sentinel end() const { return {}; }

   private:
    iterator first_;
  };

  csv_source(const std::string& path) : file_(path) {}

  std::vector<std::string> header() const;
  view read() const { 
    return make_csv_iterator(begin(file_), end(file_)); 
  }
  view read(skip_header_t) const {
    return make_csv_iterator(begin(file_), end(file_), skip_header); 
  }

 private:
  boost::iostreams::mapped_file_source file_;
};

}

BOOST_FUSION_ADAPT_STRUCT(
  io::csv_date,
  year,
  month,
  day,
  hour,
  min,
  sec
)
