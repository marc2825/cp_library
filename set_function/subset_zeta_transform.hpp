#pragma once
#include <vector>
#include "detail/transform.hpp"

namespace cp {
// f[S] <- Σ_{T⊆S} f[T]。in-place、長さは正の2冪。標準整数は加算超過を検査。
template <class T> void subset_zeta_transform(std::vector<T>& f) {
    set_function_detail::transform<true, false>(f);
}
} // namespace cp
