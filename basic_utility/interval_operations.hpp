#pragma once
#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>

namespace cp {
// 半開区間。空区間は捨て、接する区間もまとめる。
template<class T> std::vector<std::pair<T,T>> merge_intervals(std::vector<std::pair<T,T>> a) {
    std::sort(a.begin(), a.end());
    std::vector<std::pair<T,T>> result;
    for (const auto& p : a) {
        assert(!(p.second < p.first));
        if (!(p.first < p.second)) continue;
        if (result.empty() || result.back().second < p.first) result.push_back(p);
        else if (result.back().second < p.second) result.back().second = p.second;
    }
    return result;
}
// 内部の分割点のみ使用。重複・端点・区間外の点は無視。
template<class T> std::vector<std::pair<T,T>> split_interval(T l, T r, std::vector<T> cuts) {
    assert(!(r < l));
    std::sort(cuts.begin(), cuts.end());
    std::vector<std::pair<T,T>> result;
    for (const auto& x : cuts) if (l < x && x < r) { result.emplace_back(l,x); l=x; }
    if (l < r) result.emplace_back(l,r);
    return result;
}
} // namespace cp
