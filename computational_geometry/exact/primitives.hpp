#pragma once
#include <algorithm>
#include <optional>
#include "point.hpp"
namespace cp { namespace geometry { namespace exact {
struct Segment {Point a,b;};
struct Line {Point a,b;Line(Point a_,Point b_):a(a_),b(b_){if(a==b)throw std::invalid_argument("line needs two distinct points");}};
inline void validate(const Line& l){if(l.a==l.b)throw std::invalid_argument("line needs two distinct points");}
inline bool on_segment(Segment s,Point p){return !orientation(s.a,s.b,p)&&std::min(s.a.x,s.b.x)<=p.x&&p.x<=std::max(s.a.x,s.b.x)&&std::min(s.a.y,s.b.y)<=p.y&&p.y<=std::max(s.a.y,s.b.y);}
inline Ccw ccw(Point a,Point b,Point p){const int turn=orientation(a,b,p);if(turn)return turn>0?Ccw::counter_clockwise:Ccw::clockwise;const auto dx=arithmetic::sub(b.x,a.x),dy=arithmetic::sub(b.y,a.y),px=arithmetic::sub(p.x,a.x),py=arithmetic::sub(p.y,a.y);if(arithmetic::add(arithmetic::mul(dx,px),arithmetic::mul(dy,py))<0)return Ccw::online_back;return squared_distance(a,p)>squared_distance(a,b)?Ccw::online_front:Ccw::on_segment;}
inline bool segments_intersect(Segment a,Segment b){if(std::max(std::min(a.a.x,a.b.x),std::min(b.a.x,b.b.x))>std::min(std::max(a.a.x,a.b.x),std::max(b.a.x,b.b.x))||std::max(std::min(a.a.y,a.b.y),std::min(b.a.y,b.b.y))>std::min(std::max(a.a.y,a.b.y),std::max(b.a.y,b.b.y)))return false;return orientation(a.a,a.b,b.a)*orientation(a.a,a.b,b.b)<=0&&orientation(b.a,b.b,a.a)*orientation(b.a,b.b,a.b)<=0;}
struct LineIntersection {LineRelation relation;std::optional<RationalPoint> point;};
inline LineIntersection line_intersection(const Line&a,const Line&b){validate(a);validate(b);const RationalPoint p(a.a),q(b.a),r=RationalPoint(a.b)-p,s=RationalPoint(b.b)-q;const auto denominator=cross(r,s);if(!denominator.sign())return {cross(q-p,r).sign()?LineRelation::parallel:LineRelation::coincident,std::nullopt};return {LineRelation::unique,p+r*(cross(q-p,s)/denominator)};}
inline RationalPoint projection(const Line&l,Point p){validate(l);const auto dx=arithmetic::sub(l.b.x,l.a.x),dy=arithmetic::sub(l.b.y,l.a.y);const auto numerator=arithmetic::add(arithmetic::mul(arithmetic::sub(p.x,l.a.x),dx),arithmetic::mul(arithmetic::sub(p.y,l.a.y),dy));const Rational t(numerator,squared_distance(l.a,l.b));return {Rational(l.a.x)+Rational(dx)*t,Rational(l.a.y)+Rational(dy)*t};}
inline Rational squared_distance(const Line&l,Point p){validate(l);const auto c=cross(l.a,l.b,p);return {arithmetic::mul(c,c),squared_distance(l.a,l.b)};}
inline Rational squared_distance(Segment s,Point p){if(s.a==s.b)return Rational(squared_distance(s.a,p));const auto dx=arithmetic::sub(s.b.x,s.a.x),dy=arithmetic::sub(s.b.y,s.a.y),numerator=arithmetic::add(arithmetic::mul(arithmetic::sub(p.x,s.a.x),dx),arithmetic::mul(arithmetic::sub(p.y,s.a.y),dy)),denominator=squared_distance(s.a,s.b);if(numerator<=0)return Rational(squared_distance(s.a,p));if(numerator>=denominator)return Rational(squared_distance(s.b,p));const auto c=cross(s.a,s.b,p);return {arithmetic::mul(c,c),denominator};}
inline Rational squared_distance(Segment a,Segment b){if(segments_intersect(a,b))return Rational(0);return std::min({squared_distance(a,b.a),squared_distance(a,b.b),squared_distance(b,a.a),squared_distance(b,a.b)});}
// 対称な引数順と、直線同士／直線・線分の距離。
inline Rational squared_distance(Point p,const Line&l){return squared_distance(l,p);}
inline Rational squared_distance(Point p,Segment s){return squared_distance(s,p);}
inline Rational squared_distance(const Line&a,const Line&b){validate(a);validate(b);const auto r=RationalPoint(a.b)-RationalPoint(a.a),s=RationalPoint(b.b)-RationalPoint(b.a);return cross(r,s).sign()?Rational(0):squared_distance(a,b.a);}
inline Rational squared_distance(const Line&l,Segment s){validate(l);const int a=orientation(l.a,l.b,s.a),b=orientation(l.a,l.b,s.b);return a*b<=0?Rational(0):std::min(squared_distance(l,s.a),squared_distance(l,s.b));}
inline Rational squared_distance(Segment s,const Line&l){return squared_distance(l,s);}
} } } // namespace cp::geometry::exact
