#pragma once
// 説明: docs/tree/dynamic_tree_diameter.md
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>
#include "static_top_tree.hpp"
#include "../graph/detail/distance.hpp"
namespace cp {
template<class T> struct DynamicTreeDiameterResult {T length=T(0);int first=-1,second=-1;};
template<class T> struct TreeDiameterSpec {
    using Vertex=char;using Diameter=DynamicTreeDiameterResult<T>;
    struct Far{T distance;int vertex;};
    struct Point{std::optional<Far> height;std::optional<Diameter> diameter;};
    struct Path{T length;Far top,bottom;Diameter diameter;};
    static T plus(T a,T b){return detail::distance_add(a,b);}
    static Far far(Far a,Far b){return a.distance!=b.distance?(a.distance>b.distance?a:b):(a.vertex<b.vertex?a:b);}
    static Diameter pair(T d,int a,int b){if(a>b)std::swap(a,b);return {d,a,b};}
    static Diameter best(Diameter a,Diameter b){return a.length!=b.length?(a.length>b.length?a:b):(std::make_pair(a.first,a.second)<std::make_pair(b.first,b.second)?a:b);}
    Point point_identity()const{return {};}
    Point rake(Point a,Point b)const{if(!a.height)return b;if(!b.height)return a;return {far(*a.height,*b.height),best(best(*a.diameter,*b.diameter),pair(plus(a.height->distance,b.height->distance),a.height->vertex,b.height->vertex))};}
    Path add_vertex(Point p,char,int v)const{Far f{T(0),v};Diameter d{T(0),v,v};if(p.height){f=far(f,*p.height);d=best(best(d,*p.diameter),pair(p.height->distance,v,p.height->vertex));}return {T(0),f,f,d};}
    Point to_point(Path p,Edge<T> edge)const{return {Far{plus(p.top.distance,edge.cost),p.top.vertex},p.diameter};}
    Path compress(Path a,Path b,Edge<T> edge)const{return {plus(plus(a.length,edge.cost),b.length),far(a.top,{plus(plus(a.length,edge.cost),b.top.distance),b.top.vertex}),far(b.bottom,{plus(plus(b.length,edge.cost),a.bottom.distance),a.bottom.vertex}),best(best(a.diameter,b.diameter),pair(plus(plus(a.bottom.distance,edge.cost),b.top.distance),a.bottom.vertex,b.top.vertex))};}
};
// topology固定、非負の辺重みを更新。link/cutはEulerTourTree/LinkCutTreeを参照。
template<class T=long long> class DynamicTreeDiameter {
    StaticTopTreeDP<TreeDiameterSpec<T>,T> dp_;
    static const Graph<T>& checked(const Graph<T>& g){for(auto e:g.edges())if(e.cost<T(0))throw std::invalid_argument("negative dynamic diameter edge");return g;}
public:
    explicit DynamicTreeDiameter(const Graph<T>& g):dp_(checked(g),std::vector<char>(g.size(),0)){}
    void set_edge(int id,T cost){if(cost<T(0))throw std::invalid_argument("negative dynamic diameter edge");dp_.set_edge_cost(id,cost);}
    DynamicTreeDiameterResult<T> component_diameter(int v)const{return dp_.component_prod(v).diameter;}
    std::optional<DynamicTreeDiameterResult<T>> diameter()const{std::optional<DynamicTreeDiameterResult<T>> answer;for(int r:dp_.topology().roots){auto d=dp_.component_prod(dp_.topology().nodes[r].top).diameter;answer=answer?TreeDiameterSpec<T>::best(*answer,d):d;}return answer;}
};
} // namespace cp
