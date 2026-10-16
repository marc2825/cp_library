#pragma once
#include <utility>
#include <vector>
#include "polygon.hpp"
namespace cp { namespace geometry {
namespace exact {
// 有向直線の左側（境界込み）を残す。次の切断にもRationalPointのまま渡せる。
inline std::vector<RationalPoint> convex_polygon_cut(const std::vector<RationalPoint>&p,const Line&line){validate(line);if(p.empty())return {};const RationalPoint a(line.a),d=RationalPoint(line.b)-a;const auto side=[&](const RationalPoint&q){return cross(d,q-a);};std::vector<RationalPoint>out;const auto append=[&](RationalPoint q){if(out.empty()||!(out.back()==q))out.push_back(std::move(q));};for(std::size_t i=0;i<p.size();++i){const auto&u=p[i];const auto&v=p[(i+1)%p.size()];const auto s=side(u),t=side(v);if(s.sign()>=0)append(u);if(s.sign()*t.sign()<0)append(u+(v-u)*(s/(s-t)));}if(out.size()>1&&out.front()==out.back())out.pop_back();return out;}
inline std::vector<RationalPoint> convex_polygon_cut(const std::vector<Point>&p,const Line&line){std::vector<RationalPoint>q;for(auto a:p)q.emplace_back(a);return convex_polygon_cut(q,line);}
}
namespace floating {
inline std::vector<Point> convex_polygon_cut(const std::vector<Point>&p,const Line&line,Tolerance tolerance=Tolerance()){validate(line);for(auto a:p)validate(a);std::vector<Point>out;const auto append=[&](Point q){if(out.empty()||!almost_equal(out.back(),q,tolerance))out.push_back(q);};for(std::size_t i=0;i<p.size();++i){const auto u=p[i],v=p[(i+1)%p.size()];const auto s=cross(line.a,line.b,u),t=cross(line.a,line.b,v);if(tolerance.sign(s)>=0)append(u);if(tolerance.sign(s)*tolerance.sign(t)<0)append(u+(v-u)*finite(s/finite(s-t)));}if(out.size()>1&&almost_equal(out.front(),out.back(),tolerance))out.pop_back();return out;}
}
} }
