#pragma once

#include <tuple>
#include <utility>
#include <type_traits>

#include "meta.hpp"

namespace detail {

template <size_t i = 0,
          class Fun,
          class Tuple,
          size_t N = std::tuple_size<typename std::decay<Tuple>::type>::value,
          typename std::enable_if<i == N>::type* = nullptr>
void tuple_for_each(Tuple&& t, Fun f) {}
template <size_t i = 0,
          class Fun,
          class Tuple,
          size_t N = std::tuple_size<typename std::decay<Tuple>::type>::value,
          typename std::enable_if<i != N>::type* = nullptr>
void tuple_for_each(Tuple&& t, Fun f) {
  f(std::get<i>(std::forward<Tuple>(t)));
  detail::tuple_for_each<i+1>(std::forward<Tuple>(t), std::move(f));
}

}

template <class Fun, class Tuple>
void tuple_for_each(Tuple&& t, Fun f) {
  detail::tuple_for_each<0>(std::forward<Tuple>(t), std::move(f));
}

namespace detail {

template <class R, class Tuple, std::size_t... indices>
R make_struct(Tuple&& t, meta::index_sequence<indices...>) {
  return {std::get<indices>(std::forward<Tuple>(t))...};
}

}

template <class R, class Tuple>
R make_struct(Tuple&& t) {
  return detail::make_struct<R>(std::forward<Tuple>(t),
    meta::make_index_sequence<std::tuple_size<typename std::decay<Tuple>::type>::value>());
}