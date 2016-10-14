#pragma once

#include <string>

#include <boost/iterator/iterator_facade.hpp>

#include "iterator.hpp"
#include "utility.hpp"
#include "mapped_file.hpp"

namespace io {

namespace csv {

struct date {
  int year;
  int month;
  int day;
  int hour;
  int min;
  int sec;
};

namespace detail {

template <class It>
struct date_parser : qi::grammar<It, date()> {
  date_parser() : date_parser::base_type(start) {
    start %= qi::lexeme[qi::int_ >> '-' >> qi::int_ >> '-' >> qi::int_ >> ' ' >>
                        qi::int_ >> ':' >> qi::int_ >> ':' >> qi::int_];
  }
  qi::rule<It, date()> start;
};

} // namespace detail

template <class It>
class row {
 public:
  class reference {
   public:
    reference(It first, It last)
        : first_(first), last_(last) {}

    operator int() const { return extract<int>(*this); }
    operator double() const { return extract<double>(*this); }
    operator date() const { return extract<date, detail::date_parser<It>>(*this); }
    operator std::string() const { return std::string(first_, last_); }

    It begin() const { return first_; }
    It end() const { return last_; }

   private:
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
    friend class io::default_sentinel;
    
    reference dereference() const {
      return {current_, next_};
    }
    void increment() {
      current_ = next_;
      next_ = std::find_if(next_, last_, 
        [](char v){return iscomma(v) || iseol(v);});
      if (iscomma(*next_)) ++next_;
    }
    bool equal(default_sentinel that) const {
      return current_ == next_;
    }

    It current_;
    It next_;
    It last_;
  };

  row() = default;
  row(It first, It last)
      : first_(first), last_(last) {}

  iterator begin() const { return {first_, last_}; }
  default_sentinel end() const { return {}; }

 private:
  It first_ = nullptr;
  It last_ = nullptr;
};

template <class It>
class iterator :
    public boost::iterator_facade<iterator<It>, row<It>,
                                  std::forward_iterator_tag,
                                  row<It>> {
 public:
  iterator() = default;
  iterator(It first, It last)
      : current_(first), last_(last) {}
 
 private:
  friend class boost::iterator_core_access;
  friend class io::default_sentinel;
  
  row<It> dereference() const {
    return {current_, last_};
  }
  void increment() {
    current_ = std::find_if(current_, last_, 
      [](char v){return iseol(v);});
    if (current_ != last_) ++current_;
  }
  bool equal(default_sentinel that) const {
    return current_ == last_;
  }
  
  It current_ = nullptr;
  It last_ = nullptr;
};

template <class It>
class view {
 public:
  using iterator = csv::iterator<It>;

  view(iterator data) : first_(data) {}
  view(It first, It last) : first_(first, last) {}

  std::vector<std::string> fiels() const;
  view& skip_header() { 
    ++first_;
    return *this;
  }

  iterator begin() const { return first_; }
  default_sentinel end() const { return {}; }

 private:
  iterator first_;
};

} // namespace csv

template <class It>
csv::iterator<It> make_csv_iterator(It first, It last) {
  return {first, last};
}

template <class Rng>
csv::view<iterator_type<const Rng&>> make_csv_view(const Rng& rng) {
  using std::begin; using std::end;
  return {begin(rng), end(rng)};
}

class csv_source {
 public:
  csv_source(const std::string& path)
      : file_(path) {}

  csv::view<const char*> operator()() const {
    return make_csv_view(file_);
  }

private:
  boost::iostreams::mapped_file_source file_;
};

} // namespace io

BOOST_FUSION_ADAPT_STRUCT(
  io::csv::date,
  year,
  month,
  day,
  hour,
  min,
  sec
)