#pragma once
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../number_theory_convolution/detail/arithmetic.hpp"

namespace cp {
// 問い合わせ座標を先に登録。直線/線分の追加と最小値、最大値はMinimize=false。
// 係数と座標はlong long、積と和の超過はoverflow_error。
template<bool Minimize=true> class LiChaoTree {
    struct Line { long long a,b; long long eval(long long x) const { auto y=detail::arithmetic_multiply(a,x); detail::arithmetic_add_assign(y,b); return y; } };
    std::vector<long long> xs_; std::vector<std::optional<Line>> tree_;
    static bool better(long long a,long long b) { return Minimize?a<b:a>b; }
    void insert(int k,int l,int r,Line line) {
        if(!tree_[k]) { tree_[k]=line; return; }
        int m=l+(r-l)/2;
        bool left=better(line.eval(xs_[l]),tree_[k]->eval(xs_[l])),mid=better(line.eval(xs_[m]),tree_[k]->eval(xs_[m]));
        if(mid) std::swap(line,*tree_[k]);
        if(r-l==1) return;
        if(left!=mid) insert(2*k,l,m,line); else insert(2*k+1,m,r,line);
    }
    void segment(int k,int l,int r,int ql,int qr,Line line) {
        if(qr<=l || r<=ql) return;
        if(ql<=l && r<=qr) { insert(k,l,r,line); return; }
        int m=l+(r-l)/2; segment(2*k,l,m,ql,qr,line); segment(2*k+1,m,r,ql,qr,line);
    }
public:
    explicit LiChaoTree(std::vector<long long> xs):xs_(std::move(xs)) { std::sort(xs_.begin(),xs_.end()); xs_.erase(std::unique(xs_.begin(),xs_.end()),xs_.end()); tree_.resize(std::max(std::size_t(1),4*xs_.size())); }
    void add_line(long long a,long long b) { if(!xs_.empty()) insert(1,0,int(xs_.size()),{a,b}); }
    void add_segment(long long left,long long right,long long a,long long b) {
        if(left>=right || xs_.empty()) return;
        int l=int(std::lower_bound(xs_.begin(),xs_.end(),left)-xs_.begin()),r=int(std::lower_bound(xs_.begin(),xs_.end(),right)-xs_.begin());
        segment(1,0,int(xs_.size()),l,r,{a,b});
    }
    std::optional<long long> query(long long x) const {
        auto it=std::lower_bound(xs_.begin(),xs_.end(),x); if(it==xs_.end() || *it!=x) throw std::out_of_range("unregistered Li Chao coordinate");
        int p=int(it-xs_.begin()),k=1,l=0,r=int(xs_.size()); std::optional<long long> result;
        for(;;) {
            if(tree_[k]) { auto y=tree_[k]->eval(x); if(!result || better(y,*result)) result=y; }
            if(r-l==1) return result;
            int m=l+(r-l)/2; if(p<m) { k*=2; r=m; } else { k=2*k+1; l=m; }
        }
    }
};
} // namespace cp
