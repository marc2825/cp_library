#pragma once
#include <algorithm>
#include <cstddef>
#include <vector>
#include "convex_hull.hpp"

namespace cp { namespace geometry { namespace exact {
// 入力点群の凸包同士のMinkowski sum。strict CCW凸包を返す。
inline std::vector<Point> minkowski_sum(std::vector<Point>a,std::vector<Point>b){
    a=convex_hull(std::move(a));b=convex_hull(std::move(b));if(a.empty()||b.empty())return {};
    if(a.size()<3||b.size()<3){std::vector<Point> sum;for(auto x:a)for(auto y:b)sum.push_back(x+y);return convex_hull(std::move(sum));}
    const auto low=[](Point x,Point y){return x.y!=y.y?x.y<y.y:x.x<y.x;};
    std::rotate(a.begin(),std::min_element(a.begin(),a.end(),low),a.end());std::rotate(b.begin(),std::min_element(b.begin(),b.end(),low),b.end());
    const auto n=a.size(),m=b.size();std::size_t i=0,j=0;Point current=a[0]+b[0];std::vector<Point> sum{current};
    while(i<n||j<m){
        const Point u=i<n?a[(i+1)%n]-a[i]:Point{},v=j<m?b[(j+1)%m]-b[j]:Point{};
        const int turn=i<n&&j<m?arithmetic::sign(cross(u,v)):0;
        const bool take_a=i<n&&(j==m||turn>=0),take_b=j<m&&(i==n||turn<=0);
        if(take_a){current=current+u;++i;}
        if(take_b){current=current+v;++j;}
        // 片方を使い切った時は残る方向を進める。
        sum.push_back(current);
    }
    if(sum.size()>1&&sum.front()==sum.back())sum.pop_back();
    return convex_hull(std::move(sum));
}
} } } // namespace cp::geometry::exact
