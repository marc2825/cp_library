#pragma once
// 説明: docs/range_query/rollback_segtree.md
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

namespace cp {
template<class S,auto op,auto e> class RollbackSegtree {
    int n_,base_=1; std::vector<S> data_;
    std::vector<std::pair<int,S>> history_;
public:
    using Snapshot=std::size_t;
    explicit RollbackSegtree(const std::vector<S>& values={}) : n_(int(values.size())) {
        while(base_<n_) base_*=2;
        data_.assign(2*base_,e());
        for(int i=0;i<n_;++i) data_[base_+i]=values[i];
        for(int i=base_-1;i;--i) data_[i]=op(data_[2*i],data_[2*i+1]);
    }
    int size() const { return n_; }
    Snapshot snapshot() const { return history_.size(); }
    void rollback(Snapshot state) {
        assert(state<=history_.size());
        while(history_.size()>state) { auto old=history_.back(); history_.pop_back(); data_[old.first]=old.second; }
    }
    void set(int p,S value) {
        assert(0<=p && p<n_); p+=base_;
        for(;;p/=2) {
            history_.emplace_back(p,data_[p]); data_[p]=value;
            if(p==1) break;
            value=(p&1)?op(data_[p-1],data_[p]):op(data_[p],data_[p+1]);
        }
    }
    S get(int p) const { assert(0<=p && p<n_); return data_[base_+p]; }
    S prod(int l,int r) const {
        assert(0<=l && l<=r && r<=n_); S a=e(),b=e();
        for(l+=base_,r+=base_;l<r;l/=2,r/=2) {
            if(l&1) a=op(a,data_[l++]);
            if(r&1) b=op(data_[--r],b);
        }
        return op(a,b);
    }
    S all_prod() const { return data_[1]; }
};
} // namespace cp
