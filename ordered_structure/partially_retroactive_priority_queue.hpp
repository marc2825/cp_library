#pragma once
#include <algorithm>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../range_query/lazy_segtree.hpp"

namespace cp {
namespace detail {
inline int retro_pq_min(int a,int b) { return std::min(a,b); }
inline int retro_pq_inf() { return std::numeric_limits<int>::max(); }
inline int retro_pq_add(int f,int x) { return x==retro_pq_inf()?x:f+x; }
inline int retro_pq_compose(int f,int g) { return f+g; }
inline int retro_pq_zero() { return 0; }
} // namespace detail
// 事前登録時刻でinsert/delete-minの追加・削除。更新O(log m)、現在のmin/sizeはO(1)。
// Demaine–Iacono–Langermanのbridge法。空へのdelete-minを含む履歴は変更せずfalse。
template<class T,class Time=long long,class Compare=std::less<T>>
class PartiallyRetroactivePriorityQueue {
    using Bridges=LazySegtree<int,detail::retro_pq_min,detail::retro_pq_inf,int,
        detail::retro_pq_add,detail::retro_pq_compose,detail::retro_pq_zero>;
    struct Operation { int type=0; std::optional<T> key; bool live=false; };
    struct Summary { int balance=0,low=0,min_live=-1,max_dead=-1; };
    std::vector<Time> times_; std::vector<Operation> operations_; std::vector<Summary> tree_;
    int base_=1; Compare compare_; Bridges bridges_;
    int index(const Time& t) const {
        auto it=std::lower_bound(times_.begin(),times_.end(),t);
        if(it==times_.end() || t<*it) throw std::invalid_argument("unregistered time");
        return int(it-times_.begin());
    }
    bool less(int a,int b) const {
        const auto& x=*operations_[a].key; const auto& y=*operations_[b].key;
        if(compare_(x,y)) return true;
        if(compare_(y,x)) return false;
        return a<b;
    }
    int minimum(int a,int b) const { return a<0?b:b<0?a:less(a,b)?a:b; }
    int maximum(int a,int b) const { return a<0?b:b<0?a:less(a,b)?b:a; }
    Summary combine(const Summary& a,const Summary& b) const {
        return {a.balance+b.balance,std::min(a.low,a.balance+b.low),
            minimum(a.min_live,b.min_live),maximum(a.max_dead,b.max_dead)};
    }
    void update(int p) {
        const auto& o=operations_[p]; Summary s; s.balance=o.type; s.low=std::min(0,o.type);
        if(o.type==1) { if(o.live) s.min_live=p; else s.max_dead=p; }
        p+=base_; tree_[p]=s;
        while((p/=2)) tree_[p]=combine(tree_[2*p],tree_[2*p+1]);
    }
    Summary query(int l,int r) const {
        Summary a,b;
        for(l+=base_,r+=base_;l<r;l/=2,r/=2) {
            if(l&1) a=combine(a,tree_[l++]);
            if(r&1) b=combine(tree_[--r],b);
        }
        return combine(a,b);
    }
    int previous_bridge(int p) { return bridges_.min_left(p+1,[](int x){return x>0;})-1; }
    int next_bridge(int p) { return bridges_.max_right(p,[](int x){return x>0;}); }
    void weight(int p,int delta) { bridges_.apply(p+1,int(times_.size())+1,delta); }
    void make_live(int p,bool live) {
        operations_[p].live=live; update(p); weight(p,live?-1:1);
    }
public:
    explicit PartiallyRetroactivePriorityQueue(std::vector<Time> times={},Compare compare=Compare{})
        :times_(std::move(times)),compare_(std::move(compare)) {
        std::sort(times_.begin(),times_.end());
        times_.erase(std::unique(times_.begin(),times_.end(),[](const Time& a,const Time& b){return !(a<b)&&!(b<a);}),times_.end());
        operations_.resize(times_.size());
        while(base_<int(times_.size())) base_*=2;
        tree_.resize(2*base_); bridges_=Bridges(std::vector<int>(times_.size()+1,0));
    }
    int size() const { return tree_[1].balance; }
    bool empty() const { return size()==0; }
    std::optional<T> min() const {
        int p=tree_[1].min_live; return p<0?std::nullopt:operations_[p].key;
    }
    bool contains_operation(const Time& time) const { return operations_[index(time)].type!=0; }
    bool insert(const Time& time,const T& key) {
        int p=index(time); if(operations_[p].type) return false;
        int bridge=previous_bridge(p),candidate=query(bridge,int(times_.size())).max_dead;
        operations_[p]={1,key,true};
        if(candidate>=0 && less(p,candidate)) {
            operations_[p].live=false; update(p); weight(p,1); make_live(candidate,true);
        } else update(p);
        return true;
    }
    bool insert_delete_min(const Time& time) {
        int p=index(time); if(operations_[p].type) return false;
        operations_[p].type=-1; update(p);
        if(tree_[1].low<0) { operations_[p].type=0; update(p); return false; }
        int candidate=query(0,next_bridge(p)).min_live;
        make_live(candidate,false); weight(p,-1); return true;
    }
    bool erase(const Time& time) {
        int p=index(time); auto old=operations_[p]; if(!old.type) return false;
        int candidate=-1;
        if(old.type==-1) candidate=query(previous_bridge(p),int(times_.size())).max_dead;
        else if(!old.live) candidate=query(0,next_bridge(p+1)).min_live;
        operations_[p].type=0; update(p);
        if(tree_[1].low<0) { operations_[p]=old; update(p); return false; }
        operations_[p]={};
        if(old.type==-1) { weight(p,1); make_live(candidate,true); }
        else if(!old.live) { weight(p,-1); make_live(candidate,false); }
        return true;
    }
};
} // namespace cp
