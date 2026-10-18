#pragma once
#include <vector>
#include "polygon.hpp"
namespace cp { namespace geometry { namespace exact {
struct LatticePolygonCount {Wide interior,boundary;};
// 非自己交差・穴なし・非退化な整数多角形が前提。閉点の再掲は可。
inline LatticePolygonCount lattice_polygon_count(std::vector<Point> polygon){
    if(polygon.size()>1&&polygon.front()==polygon.back())polygon.pop_back();
    if(polygon.size()<3)throw std::invalid_argument("Pick requires a polygon");
    const auto area=arithmetic::abs(signed_area2(polygon));if(!area)throw std::domain_error("Pick requires nonzero area");
    Wide boundary=0;
    for(std::size_t i=0;i<polygon.size();++i){auto dx=arithmetic::abs(arithmetic::sub(polygon[(i+1)%polygon.size()].x,polygon[i].x)),dy=arithmetic::abs(arithmetic::sub(polygon[(i+1)%polygon.size()].y,polygon[i].y));while(dy){const auto next=dx%dy;dx=dy;dy=next;}boundary=arithmetic::add(boundary,dx);}
    const auto twice=arithmetic::add(arithmetic::sub(area,boundary),2);if(twice<0||twice%2)throw std::domain_error("invalid lattice polygon for Pick");return {twice/2,boundary};
}
} } }
