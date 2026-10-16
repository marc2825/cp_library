#pragma once
#include <algorithm>
#include <optional>
#include "point.hpp"
namespace cp { namespace geometry { namespace floating {
struct Segment {Point a,b;};
struct Line {Point a,b;Line(Point a_,Point b_):a(a_),b(b_){validate(a);validate(b);if(a==b)throw std::invalid_argument("line needs two distinct points");}};
inline void validate(const Line&l){validate(l.a);validate(l.b);if(l.a==l.b)throw std::invalid_argument("line needs two distinct points");}
inline bool on_segment(Segment s,Point p,Tolerance t=Tolerance()){validate(s.a);validate(s.b);validate(p);return !orientation(s.a,s.b,p,t)&&std::min(s.a.x,s.b.x)<=finite(p.x+t.epsilon())&&p.x<=finite(std::max(s.a.x,s.b.x)+t.epsilon())&&std::min(s.a.y,s.b.y)<=finite(p.y+t.epsilon())&&p.y<=finite(std::max(s.a.y,s.b.y)+t.epsilon());}
inline Ccw ccw(Point a,Point b,Point p,Tolerance t=Tolerance()){const int turn=orientation(a,b,p,t);if(turn)return turn>0?Ccw::counter_clockwise:Ccw::clockwise;const auto d=b-a,q=p-a;if(dot(d,q)< -t.epsilon())return Ccw::online_back;return norm_squared(q)>finite(norm_squared(d)+t.epsilon())?Ccw::online_front:Ccw::on_segment;}
inline bool segments_intersect(Segment a,Segment b,Tolerance t=Tolerance()){for(auto p:{a.a,a.b,b.a,b.b})validate(p);if(std::max(std::min(a.a.x,a.b.x),std::min(b.a.x,b.b.x))>finite(std::min(std::max(a.a.x,a.b.x),std::max(b.a.x,b.b.x))+t.epsilon())||std::max(std::min(a.a.y,a.b.y),std::min(b.a.y,b.b.y))>finite(std::min(std::max(a.a.y,a.b.y),std::max(b.a.y,b.b.y))+t.epsilon()))return false;return orientation(a.a,a.b,b.a,t)*orientation(a.a,a.b,b.b,t)<=0&&orientation(b.a,b.b,a.a,t)*orientation(b.a,b.b,a.b,t)<=0;}
struct LineIntersection {LineRelation relation;std::optional<Point> point;};
inline LineIntersection line_intersection(const Line&a,const Line&b,Tolerance t=Tolerance()){validate(a);validate(b);const auto r=a.b-a.a,s=b.b-b.a;const auto denominator=cross(r,s);if(!t.sign(denominator))return {t.sign(cross(b.a-a.a,r))?LineRelation::parallel:LineRelation::coincident,std::nullopt};return {LineRelation::unique,a.a+r*finite(cross(b.a-a.a,s)/denominator)};}
inline Point projection(const Line&l,Point p){validate(l);validate(p);const auto d=l.b-l.a;const auto norm=norm_squared(d);if(!norm)throw std::domain_error("line direction underflowed to zero");return l.a+d*finite(dot(p-l.a,d)/norm);}
inline long double squared_distance(const Line&l,Point p){return squared_distance(projection(l,p),p);}
inline long double squared_distance(Segment s,Point p){validate(s.a);validate(s.b);validate(p);if(s.a==s.b)return squared_distance(s.a,p);const auto d=s.b-s.a;const auto norm=norm_squared(d);if(!norm)throw std::domain_error("segment direction underflowed to zero");const auto t=std::clamp(finite(dot(p-s.a,d)/norm),0.L,1.L);return squared_distance(s.a+d*t,p);}
inline long double squared_distance(Segment a,Segment b,Tolerance t=Tolerance()){if(segments_intersect(a,b,t))return 0;return std::min({squared_distance(a,b.a),squared_distance(a,b.b),squared_distance(b,a.a),squared_distance(b,a.b)});}
inline long double distance(Point a,Point b){return finite(std::sqrt(squared_distance(a,b)));}
inline long double distance(const Line&l,Point p){return finite(std::sqrt(squared_distance(l,p)));}
inline long double distance(Segment s,Point p){return finite(std::sqrt(squared_distance(s,p)));}
inline long double distance(Segment a,Segment b,Tolerance t=Tolerance()){return finite(std::sqrt(squared_distance(a,b,t)));}
inline long double squared_distance(Point p,const Line&l){return squared_distance(l,p);}
inline long double squared_distance(Point p,Segment s){return squared_distance(s,p);}
inline long double squared_distance(const Line&a,const Line&b,Tolerance t=Tolerance()){validate(a);validate(b);return t.sign(cross(a.b-a.a,b.b-b.a))?0:squared_distance(a,b.a);}
inline long double squared_distance(const Line&l,Segment s,Tolerance t=Tolerance()){validate(l);validate(s.a);validate(s.b);return orientation(l.a,l.b,s.a,t)*orientation(l.a,l.b,s.b,t)<=0?0:std::min(squared_distance(l,s.a),squared_distance(l,s.b));}
inline long double squared_distance(Segment s,const Line&l,Tolerance t=Tolerance()){return squared_distance(l,s,t);}
inline long double distance(Point p,const Line&l){return distance(l,p);}
inline long double distance(Point p,Segment s){return distance(s,p);}
inline long double distance(const Line&a,const Line&b,Tolerance t=Tolerance()){return finite(std::sqrt(squared_distance(a,b,t)));}
inline long double distance(const Line&l,Segment s,Tolerance t=Tolerance()){return finite(std::sqrt(squared_distance(l,s,t)));}
inline long double distance(Segment s,const Line&l,Tolerance t=Tolerance()){return distance(l,s,t);}
} } } // namespace cp::geometry::floating
