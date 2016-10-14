#pragma once 

#include <utility>
#include <type_traits>
#include <iterator>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

namespace io {

namespace qi = boost::spirit::qi;

bool iseol(char c) {
  return (c == '\n') || (c == '\r');
}
bool iscomma(char c) {
  return c == ',';
}

template <class T> struct default_parser {};
template <> struct default_parser<int> { using type = qi::int_type; };
template <> struct default_parser<float> { using type = qi::float_type; };
template <> struct default_parser<double> { using type = qi::double_type; };

template <class T, class P = typename default_parser<T>::type, class U>
T extract(const U value) {
  using boost::phoenix::ref; using qi::_1;
  T v = {};
  P parser;
  qi::phrase_parse(std::begin(value), std::end(value), parser[ref(v) = _1], qi::space);
  return v;
}

template <class T>
const T* as(const char* first) {
	return reinterpret_cast<const T*>(first);
}
template <class T>
const T* as(const char* first, const char* last) {
	if (std::distance(first, last) < std::ptrdiff_t(sizeof(T))) {
		return nullptr;
	}
	return as<T>(first);
}

template <class T>
T* as(char* first) {
	return reinterpret_cast<T*>(first);
}
template <class T>
T* as(char* first, char* last) {
	if (std::distance(first, last) < std::ptrdiff_t(sizeof(T))) {
		return nullptr;
	}
	return as<T>(first);
}

}