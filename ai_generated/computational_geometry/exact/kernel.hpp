#pragma once
#include "primitives.hpp"
namespace cp { namespace geometry { namespace exact {
struct Kernel {
    using P=Point;using Metric=Wide;
    void validate(Point)const{}
    int turn(Point a,Point b,Point c)const{return orientation(a,b,c);}
    Wide side(Point a,Point b,Point c)const{return cross(a,b,c);}
    Wide magnitude(Wide x)const{return arithmetic::abs(x);}
    bool same(Point a,Point b)const{return a==b;}
    bool on(Segment s,Point p)const{return on_segment(s,p);}
    Wide distance2(Point a,Point b)const{return squared_distance(a,b);}
    Wide axis2(std::int64_t a,std::int64_t b)const{const auto d=arithmetic::sub(a,b);return arithmetic::mul(d,d);}
};
} } }
