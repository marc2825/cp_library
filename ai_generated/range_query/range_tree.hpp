#pragma once
// 説明: docs/range_query/range_tree.md
#include <algorithm>
#include <cassert>
#include <numeric>
#include <utility>
#include <vector>

namespace cp {
// 座標固定、各点IDの加算/代入と矩形和。重複座標は別IDとして保持。
template<class Coordinate=long long,class Weight=long long> class RangeTree {
    int n_,base_=1; std::vector<std::pair<Coordinate,Coordinate>> points_;
    std::vector<Coordinate> xs_; std::vector<int> position_;
    std::vector<Weight> weights_;
    std::vector<std::vector<Coordinate>> ys_; std::vector<std::vector<Weight>> bit_;
    Weight prefix(int k,const Coordinate& y) const {
        int p=int(std::lower_bound(ys_[k].begin(),ys_[k].end(),y)-ys_[k].begin()); Weight s=Weight(0);
        for(;p;p-=p&-p) s=s+bit_[k][p];
        return s;
    }
public:
    explicit RangeTree(const std::vector<std::pair<Coordinate,Coordinate>>& points={}) : n_(int(points.size())),points_(points),position_(n_),weights_(n_,Weight(0)) {
        while(base_<n_) base_*=2;
        ys_.resize(2*base_); bit_.resize(2*base_);
        std::vector<int> ids(n_); std::iota(ids.begin(),ids.end(),0);
        std::sort(ids.begin(),ids.end(),[&](int a,int b){ return points[a].first<points[b].first; });
        for(int i=0;i<n_;++i) { position_[ids[i]]=i; xs_.push_back(points[ids[i]].first); ys_[base_+i].push_back(points[ids[i]].second); }
        for(int k=base_-1;k;--k) std::merge(ys_[2*k].begin(),ys_[2*k].end(),ys_[2*k+1].begin(),ys_[2*k+1].end(),std::back_inserter(ys_[k]));
        for(int k=1;k<2*base_;++k) { auto& y=ys_[k]; y.erase(std::unique(y.begin(),y.end(),[](const Coordinate& a,const Coordinate& b){return !(a<b)&&!(b<a);}),y.end()); bit_[k].assign(y.size()+1,Weight(0)); }
    }
    int size() const { return n_; }
    void add(int id,Weight delta) {
        assert(0<=id && id<n_); weights_[id]=weights_[id]+delta;
        for(int k=base_+position_[id];k;k/=2) {
            int p=int(std::lower_bound(ys_[k].begin(),ys_[k].end(),points_[id].second)-ys_[k].begin())+1;
            for(;p<int(bit_[k].size());p+=p&-p) bit_[k][p]=bit_[k][p]+delta;
        }
    }
    void set(int id,Weight x) { assert(0<=id && id<n_); add(id,x-weights_[id]); }
    Weight get(int id) const { assert(0<=id && id<n_); return weights_[id]; }
    Weight sum(Coordinate left,Coordinate down,Coordinate right,Coordinate up) const {
        if(!(left<right) || !(down<up)) return Weight(0);
        int l=int(std::lower_bound(xs_.begin(),xs_.end(),left)-xs_.begin())+base_;
        int r=int(std::lower_bound(xs_.begin(),xs_.end(),right)-xs_.begin())+base_; Weight result=Weight(0);
        auto take=[&](int k) { result=result+prefix(k,up)-prefix(k,down); };
        for(;l<r;l/=2,r/=2) { if(l&1) take(l++); if(r&1) take(--r); }
        return result;
    }
};
} // namespace cp
