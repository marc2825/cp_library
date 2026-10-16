#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>
#include <vector>

namespace cp {
// 更新の内容と配列は利用側が保持。apply/undo(update_id,l,r)は集計も修正する。
// run開始は初期配列・空区間。説明: docs/offline_query/mo_with_updates.md
class MoWithUpdates {
    struct Query { int l,r,t; }; int n_,updates_=0; std::vector<Query> queries_;
public:
    explicit MoWithUpdates(int n):n_(n) { assert(n>=0); }
    int add_update() { return updates_++; }
    int add_query(int l,int r) { assert(0<=l && l<=r && r<=n_); queries_.push_back({l,r,updates_}); return int(queries_.size())-1; }
    std::vector<int> query_order(int block=0) const {
        assert(block>=0); if(!block) block=std::max(1,int(std::pow(std::max(1,n_),2.0/3)));
        std::vector<int> order(queries_.size()); std::iota(order.begin(),order.end(),0);
        std::sort(order.begin(),order.end(),[&](int a,int b) {
            auto x=queries_[a],y=queries_[b]; int xl=x.l/block,yl=y.l/block,xr=x.r/block,yr=y.r/block;
            if(xl!=yl) return xl<yl;
            if(xr!=yr) return (xl&1)?xr>yr:xr<yr;
            if(x.t!=y.t) return (xr&1)?x.t>y.t:x.t<y.t;
            return a<b;
        }); return order;
    }
    template<class AL,class AR,class RL,class RR,class Apply,class Undo,class Answer>
    void run(AL al,AR ar,RL rl,RR rr,Apply apply,Undo undo,Answer answer,int block=0) const {
        int l=0,r=0,t=0;
        for(int id:query_order(block)) {
            auto q=queries_[id]; while(t<q.t) apply(t++,l,r); while(t>q.t) undo(--t,l,r);
            while(l>q.l) al(--l);
            while(r<q.r) ar(r++);
            while(l<q.l) rl(l++);
            while(r>q.r) rr(--r);
            answer(id);
        }
    }
    template<class Add,class Remove,class Apply,class Undo,class Answer>
    void run(Add add,Remove remove,Apply apply,Undo undo,Answer answer,int block=0) const { run(add,add,remove,remove,apply,undo,answer,block); }
};
} // namespace cp
