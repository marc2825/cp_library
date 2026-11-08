#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;namespace f=cp::geometry::floating;
e::Point read(){e::Point p;std::cin>>p.x>>p.y;return p;}
std::vector<e::Point> points(){std::size_t n;std::cin>>n;std::vector<e::Point>p;while(n--)p.push_back(read());return p;}
void output(const e::RationalPoint&p){std::cout<<p.x.str()<<' '<<p.y.str()<<' ';}
int main(){std::cout<<std::setprecision(24);std::string op;while(std::cin>>op){try{
 if(op=="R"){long long a,b,c,d;std::cin>>a>>b>>c>>d;e::Rational x(a,b),y(c,d);std::cout<<(x+y).str()<<' '<<(x-y).str()<<' '<<(x*y).str()<<' '<<(x/y).str()<<' '<<(x<y)<<'\n';}
 else if(op=="P"){auto a=read(),b=read(),c=read();std::cout<<e::to_string(e::dot(a,b))<<' '<<e::to_string(e::cross(a,b))<<' '<<e::to_string(e::cross(a,b,c))<<' '<<e::to_string(e::squared_distance(a,b))<<'\n';}
 else if(op=="L"){auto a=read(),b=read(),c=read(),d=read();auto r=e::line_intersection(e::Line(a,b),e::Line(c,d));std::cout<<static_cast<int>(r.relation)<<' ';if(r.point)output(*r.point);std::cout<<'\n';}
 else if(op=="D"){auto a=read(),b=read(),c=read(),d=read();output(e::projection(e::Line(a,b),c));std::cout<<e::squared_distance(e::Line(a,b),c).str()<<' '<<e::squared_distance(e::Segment{a,b},c).str()<<' '<<e::squared_distance(e::Segment{a,b},e::Segment{c,d}).str()<<'\n';}
 else if(op=="H"){auto h=e::convex_hull(points());std::cout<<h.size()<<' ';for(auto p:h)std::cout<<p.x<<' '<<p.y<<' ';std::cout<<'\n';}
 else if(op=="A"){auto p=e::angular_sort(points());for(auto a:p)std::cout<<a.x<<' '<<a.y<<' ';std::cout<<'\n';}
 else if(op=="C"){auto p=points();auto a=read(),b=read();auto q=e::convex_polygon_cut(p,e::Line(a,b));std::cout<<e::polygon_area(q).str()<<' '<<q.size()<<' ';for(const auto&v:q)output(v);std::cout<<'\n';}
 else if(op=="I"){auto p=points();auto q=read();std::cout<<static_cast<int>(e::point_in_polygon(p,q))<<' '<<static_cast<int>(e::ConvexPolygon(p).contains(q))<<'\n';}
 else if(op=="F"){auto p=points();auto nearest=e::closest_pair(p);auto farthest=e::convex_diameter(e::convex_hull(p));std::cout<<(nearest?e::to_string(nearest->squared_distance):"NONE")<<' '<<(farthest?e::to_string(farthest->squared_distance):"NONE")<<'\n';}
 }catch(const std::overflow_error&){std::cout<<"OVERFLOW\n";}catch(const std::invalid_argument&){std::cout<<"INVALID\n";}catch(const std::domain_error&){std::cout<<"DOMAIN\n";}}
}
