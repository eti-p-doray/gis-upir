#pragma once

#include <boost/iterator/iterator_facade.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/fusion/include/tuple.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include "iterator.hpp"
#include "tuple.hpp"

namespace qi = boost::spirit::qi;

struct date {
  int year;
  int month;
  int day;
  int hour;
  int min;
  int sec;
};
BOOST_FUSION_ADAPT_STRUCT(
  date,
  year,
  month,
  day,
  hour,
  min,
  sec
)

template <typename Iterator>
struct date_parser : qi::grammar<Iterator, date()> {
  date_parser() : date_parser::base_type(start) {
    start %= qi::lexeme[qi::int_ >> '-' >> qi::int_ >> '-' >> qi::int_ >> ' ' >>
                        qi::int_ >> ':' >> qi::int_ >> ':' >> qi::int_];
  }

  qi::rule<Iterator, date()> start;
};

template <class It, class T> struct matcher {};
template <class It> struct matcher<It, double> { using type = qi::double_type;};
template <class It> struct matcher<It, float> { using type = qi::float_type; };
template <class It> struct matcher<It, int> { using type = qi::int_type; };
template <class It> struct matcher<It, date> { using type = date_parser<It>; };

bool iseol(char c) {
  return c == '\n';
}

using ignore_t = typename std::remove_reference<decltype(std::ignore)>::type;
constexpr struct skip_header_t {} skip_header {};

template <class It>
class csv_row {
 public:
  csv_row() = default;
  csv_row(It first, It last)
      : first_(first), last_(last) {}
  
  template <class... Args>
  std::tuple<Args...> extract() const {
    std::tuple<Args...> t;
    tuple_for_each(t, parser{first_, last_});
    return t;
  }
  template <class Arg>
  Arg extract() const {
    Arg v;
    parser{first_, last_}(v);
    return v;
  }

 private:
  struct parser {
    It current;
    It last;
    template <class T>
    void operator()(T& value) {
      using boost::phoenix::ref; using qi::_1;
      typename matcher<It, T>::type m;
      qi::phrase_parse(current, last,
        m[ref(value) = _1] >> (',' | qi::eol), qi::space);
    }
    void operator()(ignore_t&) {
      while (current != last && *current != ',' && !iseol(*current)) {
        ++current;
      } ++current;
    }
  };
 
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
    while (!iseol(*current_) && current_ != last_) {
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
