#pragma once
#include <cmath>
#include <stdexcept>
#include "../common.hpp"
namespace cp { namespace geometry { namespace floating {
inline long double finite(long double x){if(!std::isfinite(x))throw std::overflow_error("floating geometry arithmetic became nonfinite");return x;}
inline void input(long double x){if(!std::isfinite(x))throw std::invalid_argument("floating geometry input must be finite");}
struct Point {
    long double x=0,y=0;
    Point()=default; Point(long double x_,long double y_):x(x_),y(y_){input(x);input(y);}
    friend Point operator+(Point a,Point b){input(a.x);input(a.y);input(b.x);input(b.y);return {finite(a.x+b.x),finite(a.y+b.y)};}
    friend Point operator-(Point a,Point b){input(a.x);input(a.y);input(b.x);input(b.y);return {finite(a.x-b.x),finite(a.y-b.y)};}
    friend Point operator*(Point a,long double k){input(a.x);input(a.y);input(k);return {finite(a.x*k),finite(a.y*k)};}
    friend Point operator/(Point a,long double k){input(a.x);input(a.y);input(k);if(!k)throw std::domain_error("point division by zero");return {finite(a.x/k),finite(a.y/k)};}
    friend bool operator==(Point a,Point b){return a.x==b.x&&a.y==b.y;} friend bool operator!=(Point a,Point b){return !(a==b);}
    friend bool operator<(Point a,Point b){return a.x!=b.x?a.x<b.x:a.y<b.y;}
};
using Vector=Point;
inline void validate(Point p){input(p.x);input(p.y);}
inline long double dot(Point a,Point b){validate(a);validate(b);return finite(finite(a.x*b.x)+finite(a.y*b.y));}
inline long double cross(Point a,Point b){validate(a);validate(b);return finite(finite(a.x*b.y)-finite(a.y*b.x));}
inline long double cross(Point a,Point b,Point c){validate(a);validate(b);validate(c);return cross(b-a,c-a);}
inline long double norm_squared(Point a){return dot(a,a);}
inline long double squared_distance(Point a,Point b){validate(a);validate(b);return norm_squared(a-b);}
class Tolerance {
    long double epsilon_;
public:
    explicit Tolerance(long double e=1e-10L):epsilon_(e){if(!std::isfinite(e)||e<0)throw std::invalid_argument("epsilon must be finite and nonnegative");}
    long double epsilon()const{return epsilon_;}
    int sign(long double x)const{finite(x);return x>epsilon_?1:x< -epsilon_?-1:0;}
};
inline int orientation(Point a,Point b,Point c,Tolerance t=Tolerance()){return t.sign(cross(a,b,c));}
inline bool almost_equal(Point a,Point b,Tolerance t=Tolerance()){validate(a);validate(b);return std::abs(finite(a.x-b.x))<=t.epsilon()&&std::abs(finite(a.y-b.y))<=t.epsilon();}
} } } // namespace cp::geometry::floating
