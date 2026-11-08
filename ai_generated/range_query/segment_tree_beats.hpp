#pragma once
#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>

namespace cp {
// chmin/chmax/add と sum/min/max。[l,r)。全中間演算が long long に収まること。
// 値と更新量の絶対値は INF 未満。説明: docs/range_query/segment_tree_beats.md
class SegmentTreeBeats {
public:
    static constexpr long long INF = (1LL << 60);
    explicit SegmentTreeBeats(const std::vector<long long>& a = {}) : n_(int(a.size())), t_(std::max(1, 4 * n_)) {
        if (n_) build(1, 0, n_, a);
    }
    explicit SegmentTreeBeats(int n) : SegmentTreeBeats(std::vector<long long>(checked_size(n))) {}
    int size() const { return n_; }
    void range_chmin(int l, int r, long long x) { bounds(l,r); assert(-INF < x && x < INF); update(1,0,n_,l,r,x,0); }
    void range_chmax(int l, int r, long long x) { bounds(l,r); assert(-INF < x && x < INF); update(1,0,n_,l,r,x,1); }
    void range_add(int l, int r, long long x) { bounds(l,r); update(1,0,n_,l,r,x,2); }
    long long range_sum(int l, int r) { bounds(l,r); return query(1,0,n_,l,r,0); }
    long long range_min(int l, int r) { bounds(l,r); return query(1,0,n_,l,r,1); }
    long long range_max(int l, int r) { bounds(l,r); return query(1,0,n_,l,r,2); }
    long long get(int p) { assert(0 <= p && p < n_); return range_sum(p,p+1); }
private:
    struct Node { long long sum=0, hi=-INF, hi2=-INF, lo=INF, lo2=INF, add=0; int hc=0, lc=0, len=0; };
    int n_; std::vector<Node> t_;
    static int checked_size(int n) { assert(n>=0); return n; }
    void bounds(int l,int r) const { assert(0<=l && l<=r && r<=n_); (void)l; (void)r; }
    void pull(int k) {
        const auto &a=t_[2*k], &b=t_[2*k+1]; auto &s=t_[k];
        s.len=a.len+b.len; s.sum=a.sum+b.sum;
        s.hi=std::max(a.hi,b.hi); s.hc=(a.hi==s.hi?a.hc:0)+(b.hi==s.hi?b.hc:0);
        s.hi2=std::max(a.hi==s.hi?a.hi2:a.hi,b.hi==s.hi?b.hi2:b.hi);
        s.lo=std::min(a.lo,b.lo); s.lc=(a.lo==s.lo?a.lc:0)+(b.lo==s.lo?b.lc:0);
        s.lo2=std::min(a.lo==s.lo?a.lo2:a.lo,b.lo==s.lo?b.lo2:b.lo);
    }
    void build(int k,int l,int r,const std::vector<long long>& a) {
        if(r-l==1) { assert(-INF<a[l] && a[l]<INF); auto &s=t_[k]; s.sum=s.hi=s.lo=a[l]; s.hc=s.lc=s.len=1; return; }
        int m=l+(r-l)/2; build(2*k,l,m,a); build(2*k+1,m,r,a); pull(k);
    }
    void add(int k,long long x) {
        auto &s=t_[k]; s.sum+=x*s.len; s.hi+=x; s.lo+=x;
        if(s.hi2!=-INF) s.hi2+=x;
        if(s.lo2!=INF) s.lo2+=x;
        // 親のcap/floorが未伝播の子では、add直後だけ旧極値がINFを越え得る。
        // push内で続けて親の上下限を反映するため、ここで論理値の範囲を検査しない。
        s.add+=x;
    }
    void cap(int k,long long x) {
        auto &s=t_[k]; if(s.hi<=x) return;
        s.sum+=(x-s.hi)*s.hc;
        if(s.lo==s.hi) s.lo=x; else if(s.lo2==s.hi) s.lo2=x;
        s.hi=x;
    }
    void floor(int k,long long x) {
        auto &s=t_[k]; if(s.lo>=x) return;
        s.sum+=(x-s.lo)*s.lc;
        if(s.hi==s.lo) s.hi=x; else if(s.hi2==s.lo) s.hi2=x;
        s.lo=x;
    }
    void push(int k) {
        auto &s=t_[k];
        if(s.add) { add(2*k,s.add); add(2*k+1,s.add); s.add=0; }
        for(int c:{2*k,2*k+1}) { cap(c,s.hi); floor(c,s.lo); }
    }
    void update(int k,int l,int r,int ql,int qr,long long x,int op) {
        if(qr<=l || r<=ql) return;
        auto &s=t_[k];
        if(op==0 && s.hi<=x) return;
        if(op==1 && s.lo>=x) return;
        if(ql<=l && r<=qr) {
            if(op==2) { add(k,x); return; }
            if(op==0 && s.hi2<x) { cap(k,x); return; }
            if(op==1 && x<s.lo2) { floor(k,x); return; }
        }
        push(k); int m=l+(r-l)/2; update(2*k,l,m,ql,qr,x,op); update(2*k+1,m,r,ql,qr,x,op); pull(k);
    }
    long long query(int k,int l,int r,int ql,int qr,int op) {
        if(qr<=l || r<=ql) return op==0?0:op==1?INF:-INF;
        if(ql<=l && r<=qr) return op==0?t_[k].sum:op==1?t_[k].lo:t_[k].hi;
        push(k); int m=l+(r-l)/2;
        auto a=query(2*k,l,m,ql,qr,op), b=query(2*k+1,m,r,ql,qr,op);
        return op==0?a+b:op==1?std::min(a,b):std::max(a,b);
    }
};
} // namespace cp
