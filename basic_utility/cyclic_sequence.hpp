#pragma once
#include <algorithm>
#include <cassert>
#include <vector>

namespace cp {
// 循環添字・左回転・循環区間の切出し。空列の回転も可。
inline long long cyclic_index(long long i, long long n) {
    assert(n > 0);
    i %= n;
    return i < 0 ? i + n : i;
}
template<class T> void rotate_left(std::vector<T>& a, long long shift) {
    if (!a.empty()) std::rotate(a.begin(), a.begin() + cyclic_index(shift, a.size()), a.end());
}
template<class T> std::vector<T> cyclic_slice(const std::vector<T>& a, long long start, int count) {
    assert(count >= 0 && (count == 0 || !a.empty()));
    std::vector<T> result;
    if (count == 0) return result;
    auto p = static_cast<std::size_t>(cyclic_index(start, a.size()));
    for (int i = 0; i < count; ++i) {
        result.push_back(a[p]);
        if (++p == a.size()) p = 0;
    }
    return result;
}
} // namespace cp
