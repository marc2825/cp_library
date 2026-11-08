#pragma once
// 説明: docs/offline_query/rectangle_add_rectangle_sum.md
#include <array>
#include <utility>
#include <vector>
#include "compressed_fenwick_tree_2d.hpp"

namespace cp {
// 整数格子のセルへの矩形加算と矩形和。更新矩形の四隅を事前登録。
// 座標はlong long、Weightは座標から構築できる可換環。
template<class Weight=long long> class RectangleAddRectangleSum {
    using Coefficients=std::array<Weight,4>;
    // Fenwick用の加法群。
    struct Group {
        Coefficients value;
        Group(int=0) : value{Weight(0),Weight(0),Weight(0),Weight(0)} {}
        explicit Group(Coefficients x) : value(x) {}
        friend Group operator+(Group a,const Group& b) { for(int i=0;i<4;++i) a.value[i]=a.value[i]+b.value[i]; return a; }
        friend Group operator-(Group a,const Group& b) { for(int i=0;i<4;++i) a.value[i]=a.value[i]-b.value[i]; return a; }
        Group& operator+=(const Group& b) { return *this=*this+b; }
    };
    CompressedFenwickTree2D<Group,long long> bit_;
    void corner(long long x,long long y,Weight delta) {
        Weight a=Weight(x),b=Weight(y);
        bit_.add(x,y,Group({delta,Weight(0)-delta*a,Weight(0)-delta*b,delta*a*b}));
    }
    Weight prefix(long long x,long long y) const {
        auto c=bit_.sum(x,y).value; Weight a=Weight(x),b=Weight(y);
        return c[0]*a*b+c[1]*b+c[2]*a+c[3];
    }
public:
    explicit RectangleAddRectangleSum(const std::vector<std::array<long long,4>>& update_rectangles) : bit_(corners(update_rectangles)) {}
    static std::vector<std::pair<long long,long long>> corners(const std::vector<std::array<long long,4>>& rects) {
        std::vector<std::pair<long long,long long>> result;
        for(auto r:rects) if(r[0]<r[2] && r[1]<r[3]) { result.emplace_back(r[0],r[1]); result.emplace_back(r[0],r[3]); result.emplace_back(r[2],r[1]); result.emplace_back(r[2],r[3]); }
        return result;
    }
    void add(long long left,long long down,long long right,long long up,Weight value) {
        if(left>=right || down>=up) return;
        corner(left,down,value); corner(left,up,Weight(0)-value); corner(right,down,Weight(0)-value); corner(right,up,value);
    }
    Weight sum(long long left,long long down,long long right,long long up) const {
        if(left>=right || down>=up) return Weight(0);
        return prefix(right,up)-prefix(left,up)-prefix(right,down)+prefix(left,down);
    }
};
} // namespace cp
