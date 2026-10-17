#pragma once
#include <cassert>
#include <iterator>
#include <map>
#include <optional>
#include <utility>
#include <vector>

namespace cp {
// 半開区間の集合。接する区間を併合。座標の±1や番兵を使わない。
template<class T=long long> class RangeSet {
    std::map<T,T> ranges_;
public:
    bool empty() const { return ranges_.empty(); }
    int interval_count() const { return int(ranges_.size()); }
    std::vector<std::pair<T,T>> intervals() const { return {ranges_.begin(),ranges_.end()}; }
    void clear() { ranges_.clear(); }
    std::optional<std::pair<T,T>> containing(T x) const {
        auto it=ranges_.upper_bound(x);
        if(it==ranges_.begin()) return std::nullopt;
        --it; if(x<it->second) return std::make_pair(it->first,it->second);
        return std::nullopt;
    }
    bool contains(T x) const { return bool(containing(x)); }
    bool covers(T l,T r) const {
        assert(!(r<l)); if(!(l<r)) return true;
        auto p=containing(l); return p && !(p->second<r);
    }
    void insert(T l,T r) {
        assert(!(r<l)); if(!(l<r)) return;
        auto it=ranges_.lower_bound(l);
        if(it!=ranges_.begin()) { auto prev=std::prev(it); if(!(prev->second<l)) it=prev; }
        while(it!=ranges_.end() && !(r<it->first)) {
            if(it->first<l) l=it->first;
            if(r<it->second) r=it->second;
            it=ranges_.erase(it);
        }
        ranges_.emplace(l,r);
    }
    void erase(T l,T r) {
        assert(!(r<l)); if(!(l<r)) return;
        auto it=ranges_.lower_bound(l);
        if(it!=ranges_.begin()) { auto prev=std::prev(it); if(l<prev->second) it=prev; }
        while(it!=ranges_.end() && it->first<r) {
            T a=it->first,b=it->second; it=ranges_.erase(it);
            if(a<l) ranges_.emplace(a,l);
            if(r<b) { ranges_.emplace(r,b); break; }
        }
    }
};
} // namespace cp
