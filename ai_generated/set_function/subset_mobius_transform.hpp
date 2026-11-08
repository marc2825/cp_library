#pragma once
#include <vector>
#include "detail/transform.hpp"

namespace cp {
// 部分集合ゼータの逆変換。可換な加法群の-=を使う。標準整数は減算超過を検査。
template <class T> void subset_mobius_transform(std::vector<T>& f) {
    set_function_detail::transform<true, true>(f);
}
} // namespace cp
