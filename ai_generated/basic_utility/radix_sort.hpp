#pragma once
#include <array>
#include <cstddef>
#include <functional>
#include <limits>
#include <numeric>
#include <type_traits>
#include <utility>
#include <vector>
#include "detail/integer_traits.hpp"

namespace cp {
// Stable LSD byte sort. Cache each integer key once; copy records only at commit.
template <class T, class Key> void radix_sort_by(std::vector<T>& values, Key key) {
    using K = std::decay_t<std::invoke_result_t<Key&,const T&>>;
    static_assert(detail::is_integer_v<K>, "integer keys (excluding bool) required");
    using U = detail::unsigned_integer_t<K>;
    static_assert(std::is_copy_constructible_v<T>, "copyable records required");
    std::vector<U> keys; keys.reserve(values.size());
    for (const auto& value : values) {
        U k = static_cast<U>(std::invoke(key,value));
        if constexpr (std::numeric_limits<K>::is_signed) k ^= U(1) << (std::numeric_limits<U>::digits-1);
        keys.push_back(k);
    }
    std::vector<std::size_t> order(values.size()), next(values.size());
    std::iota(order.begin(),order.end(),std::size_t(0));
    for (std::size_t byte=0;byte<sizeof(K);++byte) {
        std::array<std::size_t,256> count{};
        for (auto i : order) ++count[static_cast<unsigned>((keys[i] >> (byte*8)) & U(255))];
        std::size_t offset=0;
        for (auto& n : count) { const auto old=n; n=offset; offset+=old; }
        for (auto i : order) next[count[static_cast<unsigned>((keys[i] >> (byte*8)) & U(255))]++]=i;
        order.swap(next);
    }
    std::vector<T> result; result.reserve(values.size());
    for (auto i : order) result.push_back(values[i]);
    values.swap(result);
}
template <class T> void radix_sort(std::vector<T>& values) {
    radix_sort_by(values, [](const T& value) { return value; });
}
} // namespace cp
