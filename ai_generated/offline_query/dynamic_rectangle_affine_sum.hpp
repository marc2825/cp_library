#pragma once
// 説明: docs/offline_query/dynamic_rectangle_affine_sum.md
#include <algorithm>
#include <cassert>
#include <numeric>
#include <vector>

namespace cp {
// オンライン点挿入、点代入、矩形内w<-a*w+bと矩形和。座標固定の点ID。
// 3/4を超える子はKd木を再構築。新点は過去の作用を引き継がない。
template<class Coordinate=long long,class T=long long> class DynamicRectangleAffineSum {
    struct Node {
        Coordinate x,y,xmin,xmax,ymin,ymax; T value,sum,a=T(1),b=T(0);
        int left=-1,right=-1,parent=-1,size=1;
    };
    std::vector<Node> nodes_; int root_=-1;
    int size_of(int k) const { return k<0?0:nodes_[k].size; }
    void apply_node(int k,T a,T b) {
        if(k<0) return;
        auto& n=nodes_[k]; n.value=a*n.value+b; n.sum=a*n.sum+b*T(n.size); n.a=a*n.a; n.b=a*n.b+b;
    }
    void push(int k) { T a=nodes_[k].a,b=nodes_[k].b; apply_node(nodes_[k].left,a,b); apply_node(nodes_[k].right,a,b); nodes_[k].a=T(1); nodes_[k].b=T(0); }
    void pull(int k) {
        auto& n=nodes_[k]; n.size=1; n.sum=n.value; n.xmin=n.xmax=n.x; n.ymin=n.ymax=n.y;
        for(int child:{n.left,n.right}) if(child>=0) { const auto& c=nodes_[child]; n.size+=c.size; n.sum=n.sum+c.sum; n.xmin=std::min(n.xmin,c.xmin); n.xmax=std::max(n.xmax,c.xmax); n.ymin=std::min(n.ymin,c.ymin); n.ymax=std::max(n.ymax,c.ymax); }
    }
    void flatten(int k,std::vector<int>& ids) { if(k<0) return; push(k); flatten(nodes_[k].left,ids); ids.push_back(k); flatten(nodes_[k].right,ids); }
    int build(std::vector<int>& ids,int l,int r,int depth,int parent) {
        if(l==r) return -1;
        int mid=l+(r-l)/2;
        std::nth_element(ids.begin()+l,ids.begin()+mid,ids.begin()+r,[&](int a,int b){ return depth%2?nodes_[a].y<nodes_[b].y:nodes_[a].x<nodes_[b].x; });
        int k=ids[mid]; nodes_[k].parent=parent; nodes_[k].a=T(1); nodes_[k].b=T(0);
        nodes_[k].left=build(ids,l,mid,depth+1,k); nodes_[k].right=build(ids,mid+1,r,depth+1,k); pull(k); return k;
    }
    int insert(int k,int id,int depth,int parent) {
        if(k<0) { nodes_[id].parent=parent; return id; }
        push(k);
        bool left=depth%2?nodes_[id].y<nodes_[k].y:nodes_[id].x<nodes_[k].x;
        if(left) nodes_[k].left=insert(nodes_[k].left,id,depth+1,k);
        else nodes_[k].right=insert(nodes_[k].right,id,depth+1,k);
        pull(k);
        if(4LL*std::max(size_of(nodes_[k].left),size_of(nodes_[k].right))>3LL*nodes_[k].size) { std::vector<int> ids; flatten(k,ids); return build(ids,0,int(ids.size()),depth,parent); }
        return k;
    }
    int relation(int k,Coordinate l,Coordinate d,Coordinate r,Coordinate u) const {
        const auto& n=nodes_[k];
        if(n.xmax<l || !(n.xmin<r) || n.ymax<d || !(n.ymin<u)) return 0;
        return !(n.xmin<l) && n.xmax<r && !(n.ymin<d) && n.ymax<u?2:1;
    }
    T query(int k,Coordinate l,Coordinate d,Coordinate r,Coordinate u) {
        if(k<0) return T(0);
        int type=relation(k,l,d,r,u); if(!type) return T(0); if(type==2) return nodes_[k].sum;
        push(k); const auto& n=nodes_[k]; T value=(!(n.x<l) && n.x<r && !(n.y<d) && n.y<u)?n.value:T(0);
        return value+query(n.left,l,d,r,u)+query(n.right,l,d,r,u);
    }
    void update(int k,Coordinate l,Coordinate d,Coordinate r,Coordinate u,T a,T b) {
        if(k<0) return;
        int type=relation(k,l,d,r,u); if(!type) return; if(type==2) { apply_node(k,a,b); return; }
        push(k);
        if(!(nodes_[k].x<l) && nodes_[k].x<r && !(nodes_[k].y<d) && nodes_[k].y<u) nodes_[k].value=a*nodes_[k].value+b;
        update(nodes_[k].left,l,d,r,u,a,b); update(nodes_[k].right,l,d,r,u,a,b); pull(k);
    }
    void expose(int id) { std::vector<int> path; for(int k=id;k>=0;k=nodes_[k].parent) path.push_back(k); for(auto it=path.rbegin();it!=path.rend();++it) push(*it); }
public:
    int size() const { return int(nodes_.size()); }
    int add_point(Coordinate x,Coordinate y,T value) {
        int id=size(); nodes_.push_back({x,y,x,x,y,y,value,value}); root_=insert(root_,id,0,-1); return id;
    }
    T get(int id) { assert(0<=id && id<size()); expose(id); return nodes_[id].value; }
    void set(int id,T value) { assert(0<=id && id<size()); expose(id); nodes_[id].value=value; for(int k=id;k>=0;k=nodes_[k].parent) pull(k); }
    T sum(Coordinate left,Coordinate down,Coordinate right,Coordinate up) { return !(left<right) || !(down<up)?T(0):query(root_,left,down,right,up); }
    void apply(Coordinate left,Coordinate down,Coordinate right,Coordinate up,T a,T b) { if(left<right && down<up) update(root_,left,down,right,up,a,b); }
};
} // namespace cp
