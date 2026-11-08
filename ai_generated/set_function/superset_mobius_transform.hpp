#pragma once
#include <vector>
#include "detail/transform.hpp"

namespace cp {
// 上位集合ゼータの逆変換。ビットを降順に処理して寄与を引く。
template <class T> void superset_mobius_transform(std::vector<T>& f) {
    set_function_detail::transform<false, true>(f);
}
} // namespace cp
