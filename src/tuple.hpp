#pragma once

#include <tuple>
#include <utility>
#include <type_traits>
#include <iterator>

#include "meta.hpp"

namespace detail {

template <size_t i = 0,
          class Fun,
          class Tuple,
          size_t N = std::tuple_size<typename std::decay<Tuple>::type>::value,
          typename std::enable_if<i == N>::type* = nullptr>
void tuple_for_each_(Tuple&& t, Fun f) {}
template <size_t i = 0,
          class Fun,
          class Tuple,
          size_t N = std::tuple_size<typename std::decay<Tuple>::type>::value,
          typename std::enable_if<i != N>::type* = nullptr>
void tuple_for_each_(Tuple&& t, Fun f) {
  f(std::get<i>(std::forward<Tuple>(t)));
  detail::tuple_for_each_<i+1>(std::forward<Tuple>(t), std::move(f));
}

}

template <class Fun, class Tuple>
void tuple_for_each(Tuple&& t, Fun f) {
  detail::tuple_for_each_<0>(std::forward<Tuple>(t), std::move(f));
}

namespace detail {

template <class R, class Tuple, std::size_t... indices>
R make_struct_(Tuple&& t, meta::index_sequence<indices...>) {
  return {std::get<indices>(std::forward<Tuple>(t))...};
}

}

template <class R, class Tuple>
R make_struct(Tuple&& t) {
  return detail::make_struct_<R>(std::forward<Tuple>(t),
    meta::make_index_sequence<std::tuple_size<typename std::decay<Tuple>::type>::value>());
}

using ignore_t = typename std::remove_reference<decltype(std::ignore)>::type;

namespace detail {
  
template <class It, class S>
struct extract_fn {
  It first;
  S last;
  template <class T>
  void operator()(T& val) {
    if (first == last) return;
    val = *first;
    ++first;
  }
  void operator()(ignore_t& val) {
    if (first == last) return;
    ++first;
  }
};

}

template <class... Args, class It, class S>
std::tuple<Args...> extract(It first, S last) {
  std::tuple<Args...> t;
  tuple_for_each(t, detail::extract_fn<It, S>{first, last});
  return t;
}

template <class... Args, class Rng>
std::tuple<Args...> extract(Rng&& rng) {
  return extract<Args...>(std::begin(rng), std::end(rng));
}