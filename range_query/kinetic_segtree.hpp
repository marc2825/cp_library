#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
namespace cp {
// 単調な整数時刻上の線形値 a*t+b。certificateが失効した部分だけ再計算。
class KineticSegtree {
public:
    using Wide=__int128_t;
    struct Line {long long slope,intercept;};
    struct Minimum {int index;Wide value;};
private:
    struct Node {int winner=-1;Wide expires=Wide(std::numeric_limits<long long>::max())+1;};
    std::vector<Line> lines_;
    std::vector<Node> tree_;
    long long time_=0;
    static Wide never(){return Wide(std::numeric_limits<long long>::max())+1;}
    Wide value(int i) const {return Wide(lines_[i].slope)*time_+lines_[i].intercept;}
    int better(int a,int b) const {
        if(a<0 || b<0)return a<0?b:a;
        Wide x=value(a),y=value(b);return x<y || (x==y && a<b)?a:b;
    }
    void pull(int k) {
        int a=tree_[2*k].winner,b=tree_[2*k+1].winner,w=better(a,b),loser=w==a?b:a;
        Wide expiry=std::min(tree_[2*k].expires,tree_[2*k+1].expires);
        if(loser>=0 && lines_[loser].slope<lines_[w].slope) {
            Wide gap=value(loser)-value(w),speed=Wide(lines_[w].slope)-lines_[loser].slope;
            Wide wait=gap/speed+(gap%speed!=0 || loser>w);
            expiry=std::min(expiry,Wide(time_)+wait);
        }
        tree_[k]={w,expiry};
    }
    void build(int k,int l,int r) {
        if(r-l==1){tree_[k].winner=l;return;}int m=l+(r-l)/2;build(2*k,l,m);build(2*k+1,m,r);pull(k);
    }
    void refresh(int k,int l,int r) {
        if(tree_[k].expires>time_)return;
        int m=l+(r-l)/2;refresh(2*k,l,m);refresh(2*k+1,m,r);pull(k);
    }
    void update(int k,int l,int r,int p) {
        if(r-l==1){tree_[k]={p,never()};return;}int m=l+(r-l)/2;
        if(p<m)update(2*k,l,m,p);else update(2*k+1,m,r,p);pull(k);
    }
    int query(int k,int l,int r,int a,int b) const {
        if(b<=l || r<=a)return -1;
        if(a<=l && r<=b)return tree_[k].winner;
        int m=l+(r-l)/2;return better(query(2*k,l,m,a,b),query(2*k+1,m,r,a,b));
    }
public:
    explicit KineticSegtree(std::vector<Line> lines={},long long time=0)
        : lines_(std::move(lines)),tree_(std::max(std::size_t(1),4*lines_.size())),time_(time) {
        if(time<0)throw std::invalid_argument("nonnegative time required");
        if(size())build(1,0,size());
    }
    int size() const {return int(lines_.size());}
    long long time() const {return time_;}
    void advance(long long time) {
        if(time<time_)throw std::invalid_argument("kinetic time cannot decrease");
        time_=time;if(size())refresh(1,0,size());
    }
    void set(int p,Line line) {assert(0<=p && p<size());lines_[p]=line;update(1,0,size(),p);}
    std::optional<Minimum> minimum(int l,int r) const {
        assert(0<=l && l<=r && r<=size());if(l==r)return std::nullopt;
        int w=query(1,0,size(),l,r);return Minimum{w,value(w)};
    }
    std::optional<long long> next_event() const {
        if(!size() || tree_[1].expires==never())return std::nullopt;
        return static_cast<long long>(tree_[1].expires);
    }
};
} // namespace cp
