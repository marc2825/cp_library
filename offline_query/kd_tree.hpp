#pragma once
// 説明: docs/offline_query/kd_tree.md
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

namespace cp {
// 静的な2D点。矩形報告・最近点（同距離なら最小ID）。距離はlong double。
template<class Coordinate=long long> class KdTree {
    struct Node { int id,left=-1,right=-1; Coordinate xmin,xmax,ymin,ymax; };
    std::vector<std::pair<Coordinate,Coordinate>> points_; std::vector<Node> nodes_; int root_=-1;
    int build(std::vector<int>& ids,int l,int r,int depth) {
        if(l==r) return -1;
        int mid=l+(r-l)/2;
        auto coordinate=[&](int i){return depth%2?points_[i].second:points_[i].first;};
        std::nth_element(ids.begin()+l,ids.begin()+mid,ids.begin()+r,[&](int a,int b){return coordinate(a)<coordinate(b);});
        int k=int(nodes_.size()),id=ids[mid]; auto [x,y]=points_[id]; nodes_.push_back({id,-1,-1,x,x,y,y});
        int a=build(ids,l,mid,depth+1),b=build(ids,mid+1,r,depth+1); nodes_[k].left=a; nodes_[k].right=b;
        for(int child:{a,b}) if(child>=0) { nodes_[k].xmin=std::min(nodes_[k].xmin,nodes_[child].xmin); nodes_[k].xmax=std::max(nodes_[k].xmax,nodes_[child].xmax); nodes_[k].ymin=std::min(nodes_[k].ymin,nodes_[child].ymin); nodes_[k].ymax=std::max(nodes_[k].ymax,nodes_[child].ymax); }
        return k;
    }
    static long double square(long double x) { return x*x; }
    long double box_distance(int k,Coordinate x,Coordinate y) const {
        if(k<0) return std::numeric_limits<long double>::infinity();
        const auto& n=nodes_[k];
        long double a=static_cast<long double>(x),b=static_cast<long double>(y);
        return square(std::max({static_cast<long double>(n.xmin)-a,0.L,a-static_cast<long double>(n.xmax)}))+square(std::max({static_cast<long double>(n.ymin)-b,0.L,b-static_cast<long double>(n.ymax)}));
    }
public:
    explicit KdTree(const std::vector<std::pair<Coordinate,Coordinate>>& points={}) : points_(points) { std::vector<int> ids(points.size()); std::iota(ids.begin(),ids.end(),0); root_=build(ids,0,int(ids.size()),0); }
    std::vector<int> report(Coordinate left,Coordinate down,Coordinate right,Coordinate up) const {
        std::vector<int> result; if(!(left<right) || !(down<up)) return result;
        auto visit=[&](auto&& self,int k)->void {
            if(k<0) return;
            const auto& n=nodes_[k];
            if(n.xmax<left || !(n.xmin<right) || n.ymax<down || !(n.ymin<up)) return;
            auto [x,y]=points_[n.id]; if(!(x<left) && x<right && !(y<down) && y<up) result.push_back(n.id);
            self(self,n.left); self(self,n.right);
        };
        visit(visit,root_); return result;
    }
    std::optional<std::pair<int,long double>> nearest(Coordinate x,Coordinate y) const {
        if(root_<0) return std::nullopt;
        int best=-1; long double distance=std::numeric_limits<long double>::infinity();
        auto visit=[&](auto&& self,int k)->void {
            if(k<0 || box_distance(k,x,y)>distance) return;
            const auto& n=nodes_[k]; auto [a,b]=points_[n.id];
            long double d=square(static_cast<long double>(a)-static_cast<long double>(x))+square(static_cast<long double>(b)-static_cast<long double>(y));
            if(d<distance || (d==distance && (best<0 || n.id<best))) { distance=d; best=n.id; }
            int l=n.left,r=n.right; if(box_distance(l,x,y)>box_distance(r,x,y)) std::swap(l,r);
            self(self,l); self(self,r);
        };
        visit(visit,root_); return std::make_pair(best,distance);
    }
};
} // namespace cp
