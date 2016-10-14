#include <type_traits>

namespace meta {

namespace detail {

template <bool B, class... T> struct if_t {};
template <bool B> struct if_t<B> : std::enable_if<B, int> {};
template <bool B, class T> struct if_t<B, T> : std::enable_if<B, T> {};
template <bool B, class T, class F> struct if_t<B, T, F> : std::conditional<B, T, F> {};

}

template <bool B, class... T>
using if_t = typename detail::if_t<B, T...>::type;

template <class T> struct remove_rvalue { using type = T; };
template <class T> struct remove_rvalue<T&&> { using type = T; };
template <class T> using remove_rvalue_t = typename remove_rvalue<T>::type;

template< class... >
using void_t = void;



template< class T, T... S >
struct integer_sequence {};
template<size_t... S>
using index_sequence = integer_sequence<size_t, S...>;


namespace detail {

template <class T>
struct IntegerSequenceHolder {
  template<size_t N, size_t... S>
  struct Sequence :
    Sequence<N-1, N-1, S...> {};

  template<size_t... S>
  struct Sequence<size_t(0), S...>
  { using type = integer_sequence<T, S...>; };
};

}

template <class T, T N>
using make_integer_sequence =
  typename detail::IntegerSequenceHolder<T>::template Sequence<N>::type;

template <size_t N>
using make_index_sequence =
  typename detail::IntegerSequenceHolder<size_t>::template Sequence<N>::type;
  
}