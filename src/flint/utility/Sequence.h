#pragma once
#include <utility>

namespace utility {

    namespace detail {
        template <unsigned int Count, typename T>
        struct RepeatedIntegerSequenceImpl {
            template <T V, T... Vs>
            static constexpr decltype(auto) Get(std::integer_sequence<T, V, Vs...>) {
                return RepeatedIntegerSequenceImpl<Count - 1, T>::Get(std::integer_sequence<T, V, V, Vs...>{});
            }
        };

        template <typename T>
        struct RepeatedIntegerSequenceImpl<0, T> {
            template <T V, T... Vs>
            static constexpr decltype(auto) Get(std::integer_sequence<T, V, Vs...>) {
                return std::integer_sequence<T, Vs...>{};
            }
        };
    }

template <unsigned int Count, typename T, T Value>
using RepeatedIntegerSequence = decltype(detail::RepeatedIntegerSequenceImpl<Count, T>::Get(std::integer_sequence<T, Value>{}));

}

