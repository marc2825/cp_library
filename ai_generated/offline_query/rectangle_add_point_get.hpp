#pragma once
// 説明: docs/offline_query/rectangle_add_point_get.md
#include <array>
#include <vector>
#include "compressed_fenwick_tree_2d.hpp"
#include "rectangle_add_rectangle_sum.hpp"

namespace cp {
// 四隅の差分を疎な2D BITへ置く。更新境界のみ事前登録。
template<class T=long long> class RectangleAddPointGet {
    CompressedFenwickTree2D<T,long long> bit_;
public:
    explicit RectangleAddPointGet(const std::vector<std::array<long long,4>>& update_rectangles) : bit_(RectangleAddRectangleSum<T>::corners(update_rectangles)) {}
    void add(long long left,long long down,long long right,long long up,T value) {
        if(left>=right || down>=up) return;
        bit_.add(left,down,value); bit_.add(left,up,T(0)-value); bit_.add(right,down,T(0)-value); bit_.add(right,up,value);
    }
    T get(long long x,long long y) const {
        return bit_.sum_inclusive(x,y);
    }
};
} // namespace cp
