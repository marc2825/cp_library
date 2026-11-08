#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace cp {
// 固定D次元の動的点集合。KD-treeと更新bufferを平方分割で再構築する。
// 距離はlong doubleの二乗Euclid距離。同距離は小さいID。厳密演算は保証しない。
template<std::size_t D> class DynamicNearestNeighbor {
    static_assert(D>=1 && D<=10,"dimension must be in [1,10]");
public:
    using Point=std::array<long double,D>;
    struct Result {int id;long double squared_distance;};
private:
    struct Record {Point point;bool active=true;std::size_t generation=0;};
    struct Entry {int id;std::size_t generation;Point point;};
    struct Node {Entry entry;Point lower,upper;int left=-1,right=-1;};
    std::vector<Record> records_; std::vector<Entry> buffer_; std::vector<Node> nodes_;
    int root_=-1,active_=0,updates_=0,threshold_=1;
    static void validate(const Point& p) {
        for(auto x:p) if(!std::isfinite(x)) throw std::invalid_argument("coordinates must be finite");
    }
    bool current(const Entry& e) const { return records_[e.id].active && records_[e.id].generation==e.generation; }
    static long double distance(const Point& a,const Point& b) {
        long double d=0; for(std::size_t k=0;k<D;++k) { long double x=a[k]-b[k]; d+=x*x; } return d;
    }
    int build(std::vector<Entry>& a,int l,int r,std::size_t axis) {
        if(l==r) return -1;
        int m=l+(r-l)/2;
        std::nth_element(a.begin()+l,a.begin()+m,a.begin()+r,[&](const Entry& x,const Entry& y) {
            return x.point[axis]<y.point[axis] || (x.point[axis]==y.point[axis] && x.id<y.id);
        });
        int p=int(nodes_.size()); nodes_.push_back({a[m],a[m].point,a[m].point,-1,-1});
        int left=build(a,l,m,(axis+1)%D),right=build(a,m+1,r,(axis+1)%D);
        nodes_[p].left=left; nodes_[p].right=right;
        for(int child:{left,right}) if(child>=0) for(std::size_t k=0;k<D;++k) {
            nodes_[p].lower[k]=std::min(nodes_[p].lower[k],nodes_[child].lower[k]);
            nodes_[p].upper[k]=std::max(nodes_[p].upper[k],nodes_[child].upper[k]);
        }
        return p;
    }
    long double bound(int p,const Point& query) const {
        if(p<0) return std::numeric_limits<long double>::infinity();
        long double d=0;
        for(std::size_t k=0;k<D;++k) {
            long double x=query[k]<nodes_[p].lower[k]?nodes_[p].lower[k]-query[k]:
                query[k]>nodes_[p].upper[k]?query[k]-nodes_[p].upper[k]:0;
            d+=x*x;
        }
        return d;
    }
    void consider(const Entry& e,const Point& q,std::optional<Result>& best) const {
        if(!current(e)) return;
        long double d=distance(e.point,q);
        if(!best || d<best->squared_distance || (d==best->squared_distance && e.id<best->id)) best=Result{e.id,d};
    }
    void search(int p,const Point& q,std::optional<Result>& best) const {
        if(p<0 || (best && bound(p,q)>best->squared_distance)) return;
        const auto& n=nodes_[p]; consider(n.entry,q,best);
        int a=n.left,b=n.right; if(bound(a,q)>bound(b,q)) std::swap(a,b);
        search(a,q,best); search(b,q,best);
    }
    void changed() { if(++updates_>=threshold_) rebuild(); }
public:
    DynamicNearestNeighbor()=default;
    explicit DynamicNearestNeighbor(const std::vector<Point>& points) {
        records_.reserve(points.size());
        for(const auto& p:points) { validate(p); records_.push_back({p,true,0}); }
        active_=int(points.size()); rebuild();
    }
    int size() const { return active_; }
    bool empty() const { return active_==0; }
    bool contains(int id) const { return 0<=id && id<int(records_.size()) && records_[id].active; }
    Point point(int id) const {
        if(!contains(id)) throw std::out_of_range("inactive point ID");
        return records_[id].point;
    }
    int insert(const Point& p) {
        validate(p); int id=int(records_.size()); records_.push_back({p,true,0});
        buffer_.push_back({id,0,p}); ++active_; changed(); return id;
    }
    bool erase(int id) {
        if(!contains(id)) return false;
        records_[id].active=false; --active_; changed(); return true;
    }
    bool move(int id,const Point& p) {
        validate(p); if(!contains(id)) return false;
        auto& r=records_[id]; r.point=p; ++r.generation; buffer_.push_back({id,r.generation,p}); changed(); return true;
    }
    void rebuild() {
        std::vector<Entry> entries; entries.reserve(active_);
        for(int id=0;id<int(records_.size());++id) if(records_[id].active) {
            auto& r=records_[id]; r.generation=0; entries.push_back({id,0,r.point});
        }
        nodes_.clear(); nodes_.reserve(entries.size()); root_=build(entries,0,int(entries.size()),0);
        buffer_.clear(); updates_=0; threshold_=std::max(1,int(std::sqrt(static_cast<long double>(records_.size()+1))));
    }
    std::optional<Result> nearest(const Point& q) const {
        validate(q); std::optional<Result> best;
        search(root_,q,best); for(const auto& e:buffer_) consider(e,q,best); return best;
    }
};
} // namespace cp
