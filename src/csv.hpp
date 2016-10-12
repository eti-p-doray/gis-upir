#pragma once

#include <boost/iterator/iterator_facade.hpp>
#include <boost/lexical_cast.hpp>

#include "mapped_file.hpp"
#include "iterator.hpp"
#include "tuple.hpp"
#include "utility.hpp"

namespace io {

struct csv_date {
  int year;
  int month;
  int day;
  int hour;
  int min;
  int sec;
};

template <class It>
csv_date csv_date_cast(It first, It last) {
  csv_date date;
  It next = std::find(first, last, '-');
  date.year = boost::lexical_cast<int>(first, next);
  first = next, next = std::find(first, last, '-');
  date.month = boost::lexical_cast<int>(first, next);
  first = next, next = std::find_if(first, last, std::isspace);
  date.day = boost::lexical_cast<int>(first, next);
  first = next, next = std::find(first, last, ':');
  date.hour = boost::lexical_cast<int>(first, next);
  first = next, next = std::find(first, last, ':');
  date.min = boost::lexical_cast<int>(first, next);
  date.sec = boost::lexical_cast<int>(first, last);
}

template <class It>
class csv_row {
 public:
  class reference {
   public:
    reference(It first, It last)
        : first_(first), last_(last) {}

    operator int() const { return boost::lexical_cast<int>(first_, last_); }
    operator double() const { return boost::lexical_cast<double>(first_, last_); }
    operator csv_date() const { return csv_date_cast(first_, last_); }
    operator std::string() const { return std::string(first_, last_); }

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
    friend class default_sentinel;
    
    reference dereference() const {
      return {current_, next_};
    }
    void increment() {
      current_ = next_;
      next_ = std::find_if(next_, last_, 
        [](char v){return iscomma(v) || iseol(v);})
      if (iscomma(*next_)) ++next_;
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
 
 private:
  friend class boost::iterator_core_access;
  friend class default_sentinel;
  
  csv_row<It> dereference() const {
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
csv_iterator<It> make_csv_iterator(It first, It last) {
  return {first, last};
}

template <class It>
class csv_view {
 public:
  using iterator = csv_iterator<It>;

  view(iterator data) : first_(first) {}
  view(It first, It last) : first_(first, last) {}

  csv_source(const std::string& path) : file_(path) {}

  std::vector<std::string> header() const;
  csv_view& skip_header() { 
    ++first_;
    return *this;
  }

  iterator begin() const { return first_; }
  default_sentinel end() const { return {}; }

 private:
  iterator first_;
};

template <class Rng>
csv_view<It> make_csv_view(const Rng& rng) {
  return {std::begin(rng), std::end(rng)};
}

class csv_source {
 public:

  csv_view<const char*> operator()() const { 
    return make_csv_iterator(begin(file_), end(file_)); 
  }

 private:
  boost::iostreams::mapped_file_source file_;
};

}