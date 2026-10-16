#pragma once
#include "primitives.hpp"
namespace cp { namespace geometry { namespace floating {
struct Kernel {
    using P=Point;using Metric=long double;Tolerance tolerance;
    explicit Kernel(Tolerance t=Tolerance()):tolerance(t){}
    void validate(Point p)const{floating::validate(p);}
    int turn(Point a,Point b,Point c)const{return orientation(a,b,c,tolerance);}
    long double side(Point a,Point b,Point c)const{return cross(a,b,c);}
    long double magnitude(long double x)const{return std::abs(x);}
    bool same(Point a,Point b)const{return almost_equal(a,b,tolerance);}
    bool on(Segment s,Point p)const{return on_segment(s,p,tolerance);}
    long double distance2(Point a,Point b)const{return squared_distance(a,b);}
    long double axis2(long double a,long double b)const{const auto d=finite(a-b);return finite(d*d);}
};
} } }
