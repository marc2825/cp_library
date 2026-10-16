#pragma once
#include <vector>
#include "detail/transform.hpp"

namespace cp {
// f[S] <- Σ_{T⊇S} f[T]。in-place、長さは正の2冪。
template <class T> void superset_zeta_transform(std::vector<T>& f) {
    set_function_detail::transform<false, false>(f);
}
} // namespace cp
