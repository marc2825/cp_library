#pragma once
#include <cstddef>
#include <functional>
#include <utility>
#include <vector>
#include "../range_query/segment_tree_cover.hpp"

namespace cp {
// 有効期間 [l,r) の操作を登録し、各時刻の状態を訪問後に復元する。
template<class Event> class SegmentTreeOverTime {
    SegmentTreeCover layout_;
    std::vector<Event> events_;
    std::vector<std::vector<std::size_t>> buckets_;
public:
    explicit SegmentTreeOverTime(int times) : layout_(times),buckets_(layout_.node_capacity()) {}
    int size() const { return layout_.size(); }
    std::size_t event_count() const { return events_.size(); }
    void add(int l, int r, Event event) {
        auto nodes=layout_.cover(l,r);
        if (nodes.empty()) return;
        // コピー不要な Event も格納できる。各 bucket は ID のみを保持する。
        std::size_t id=events_.size();
        events_.push_back(std::move(event));
        try { for (auto k:nodes) buckets_[k].push_back(id); }
        catch (...) {
            for (auto k:nodes) if (!buckets_[k].empty() && buckets_[k].back()==id) buckets_[k].pop_back();
            events_.pop_back(); throw;
        }
    }
    template<class State,class Apply,class Visit> void run(State& state, Apply apply, Visit visit) const {
        if (!size()) return;
        auto dfs=[&](auto&& self,std::size_t k,std::size_t l,std::size_t r)->void {
            if (l>=std::size_t(size())) return;
            auto saved=state.snapshot();
            try {
                for (auto id:buckets_[k]) std::invoke(apply,state,events_[id]);
                if (r-l==1) std::invoke(visit,state,int(l));
                else { auto m=l+(r-l)/2; self(self,2*k,l,m); self(self,2*k+1,m,r); }
            } catch (...) { state.rollback(saved); throw; }
            state.rollback(saved);
        };
        dfs(dfs,1,0,layout_.base());
    }
    template<class State,class Visit> void run(State& state, Visit visit) const {
        run(state,[](State& s,const Event& e){ s.apply(e); },std::move(visit));
    }
};
} // namespace cp
