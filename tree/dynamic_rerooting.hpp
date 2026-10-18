#pragma once
// 説明: docs/tree/dynamic_rerooting.md
#include <algorithm>
#include <utility>
#include <vector>
#include "static_top_tree.hpp"
namespace cp {
// 固定topology。Specが式の外側をContextとして伝える。queryの根は任意。
template<class Spec,class Weight=long long> class DynamicRerootingTreeDP:public StaticTopTreeDP<Spec,Weight> {
    using Base=StaticTopTreeDP<Spec,Weight>;
public:
    using Base::Base;
    auto query(int v)const {
        assert(0<=v && v<this->size());std::vector<int> chain;
        for(int p=v;p!=-1;p=this->topology_.nodes[p].parent)chain.push_back(p);
        auto context=this->spec_.root_context();using K=StaticTopTree::Kind;
        for(int j=static_cast<int>(chain.size())-1;j>0;--j){int p=chain[j],child=chain[j-1];auto node=this->topology_.nodes[p];
            if(node.kind==K::compress){if(child==node.left)context=this->spec_.compress_left_context(context,*this->paths_[node.right],this->connector(p));else context=this->spec_.compress_right_context(context,*this->paths_[node.left],this->connector(p));}
            else if(node.kind==K::rake)context=this->spec_.rake_context(context,*this->points_[child==node.left?node.right:node.left]);
            else if(node.kind==K::vertex)context=this->spec_.vertex_point_context(context,this->values_[p],p);
            else context=this->spec_.edge_context(context,this->connector(p));
        }
        return this->spec_.finish(*this->paths_[v],context);
    }
};
// 点の質量と辺長の更新、任意根からの重み付き距離和。Tの中間値は利用者が確保する。
template<class T=long long> struct TreeDistanceSumSpec {
    using Vertex=T;
    struct Point{T mass=T(0),distance=T(0);};
    struct Path{T length=T(0),mass=T(0),top_sum=T(0),bottom_sum=T(0);};
    struct Context{Point top,bottom;};
    Point point_identity()const{return {};}
    Point rake(Point a,Point b)const{return {a.mass+b.mass,a.distance+b.distance};}
    Path add_vertex(Point a,T x,int)const{return {T(0),a.mass+x,a.distance,a.distance};}
    Path compress(Path a,Path b,Edge<T> e)const{return {a.length+e.cost+b.length,a.mass+b.mass,a.top_sum+b.top_sum+b.mass*(a.length+e.cost),b.bottom_sum+a.bottom_sum+a.mass*(b.length+e.cost)};}
    Point to_point(Path p,Edge<T> e)const{return {p.mass,p.top_sum+p.mass*e.cost};}
    Context root_context()const{return {};}
    Context compress_left_context(Context c,Path right,Edge<T> e)const{return {c.top,{right.mass+c.bottom.mass,right.top_sum+right.mass*e.cost+c.bottom.distance+c.bottom.mass*(right.length+e.cost)}};}
    Context compress_right_context(Context c,Path left,Edge<T> e)const{return {{left.mass+c.top.mass,left.bottom_sum+left.mass*e.cost+c.top.distance+c.top.mass*(left.length+e.cost)},c.bottom};}
    Context rake_context(Context c,Point sibling)const{return {rake(rake(c.top,c.bottom),sibling),{}};}
    Context vertex_point_context(Context c,T value,int)const{return {{c.top.mass+c.bottom.mass+value,c.top.distance+c.bottom.distance},{}};}
    Context edge_context(Context c,Edge<T> e)const{auto p=rake(c.top,c.bottom);p.distance=p.distance+p.mass*e.cost;return {p,{}};}
    T finish(Path v,Context c)const{return v.top_sum+c.top.distance+c.bottom.distance;}
};
template<class T=long long> using DynamicTreeDistanceSum=DynamicRerootingTreeDP<TreeDistanceSumSpec<T>,T>;
} // namespace cp
