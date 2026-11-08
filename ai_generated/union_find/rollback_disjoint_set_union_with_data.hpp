#pragma once
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
#include "rollback_disjoint_set_union.hpp"

namespace cp {
// Data は snapshot()/rollback(token) を持つ借用オブジェクト。
// 任意の付随情報の履歴を DSU と同じ境界まで戻す。
template<class Data> class RollbackDisjointSetUnionWithData {
    using DataSnapshot=decltype(std::declval<const Data&>().snapshot());
    struct Change { DataSnapshot before; std::uint64_t serial; };
    RollbackDisjointSetUnion dsu_;
    Data& data_;
    std::vector<Change> history_;
    std::uint64_t serial_=0;
public:
    class Snapshot {
        friend class RollbackDisjointSetUnionWithData;
        const RollbackDisjointSetUnionWithData* owner_;
        std::size_t depth_;
        std::uint64_t tail_;
        DataSnapshot data_;
        Snapshot(const RollbackDisjointSetUnionWithData* owner,std::size_t depth,std::uint64_t tail,DataSnapshot data)
            : owner_(owner),depth_(depth),tail_(tail),data_(std::move(data)) {}
    };
    RollbackDisjointSetUnionWithData(int n,Data& data) : dsu_(n),data_(data) {}
    RollbackDisjointSetUnionWithData(const RollbackDisjointSetUnionWithData&)=delete;
    RollbackDisjointSetUnionWithData& operator=(const RollbackDisjointSetUnionWithData&)=delete;
    int size() const { return dsu_.size(); }
    int leader(int v) const { return dsu_.leader(v); }
    bool same(int u,int v) const { return dsu_.same(u,v); }
    int component_size(int v) const { return dsu_.component_size(v); }
    int component_count() const { return dsu_.component_count(); }
    Data& data() { return data_; }
    const Data& data() const { return data_; }
    Snapshot snapshot() const {
        return Snapshot(this,history_.size(),history_.empty()?0:history_.back().serial,data_.snapshot());
    }
    template<class F> bool merge(int u,int v,F&& callback) {
        if (serial_==std::numeric_limits<std::uint64_t>::max())
            throw std::length_error("RollbackDisjointSetUnionWithData revision limit");
        history_.push_back({data_.snapshot(),++serial_});
        try {
            return dsu_.merge(u,v,[&](int a,int b){ std::invoke(callback,data_,a,b); });
        } catch (...) {
            data_.rollback(history_.back().before); history_.pop_back(); throw;
        }
    }
    bool merge(int u,int v) { return merge(u,v,[](Data&,int,int){}); }
    void undo() {
        assert(!history_.empty());
        data_.rollback(history_.back().before); dsu_.undo(); history_.pop_back();
    }
    void rollback(const Snapshot& s) {
        if (s.owner_!=this || s.depth_>history_.size() ||
            (s.depth_ && history_[s.depth_-1].serial!=s.tail_))
            throw std::invalid_argument("snapshot is not on the current DSU history");
        data_.rollback(s.data_);
        dsu_.rollback(s.depth_);
        while (history_.size()>s.depth_) history_.pop_back();
    }
};
} // namespace cp
