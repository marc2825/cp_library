#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include "common.hpp"
#include "range_query/segment_tree_beats.hpp"
#include "static_query/wavelet_matrix.hpp"
#include "ordered_structure/persistent_segment_tree.hpp"
#include "ordered_structure/persistent_order_statistics.hpp"
#include "ordered_structure/implicit_treap.hpp"
#include "offline_query/all.hpp"
long long sum_op(long long a,long long b) { return a+b; }
long long zero() { return 0; }
std::string cat(std::string a,std::string b) { return a+b; }
std::string empty_string() { return {}; }
struct NonDefault { int value; NonDefault()=delete; explicit NonDefault(int x):value(x){} };
NonDefault combine(NonDefault a,NonDefault b) { return NonDefault(a.value+b.value); }
NonDefault nd_zero() { return NonDefault(0); }
std::mt19937_64 rng(20261017);
int pick(int n) { return int(rng()%n); }
void beats() {
    for(int trial=0;trial<150;++trial) {
        int n=pick(70); std::vector<long long> a(n); for(auto &x:a) x=pick(101)-50;
        cp::SegmentTreeBeats tree(a);
        for(int step=0;step<700;++step) {
            int l=pick(n+1),r=pick(n+1); if(l>r) std::swap(l,r); long long x=pick(201)-100;
            int op=pick(4); if(op==0) { tree.range_chmin(l,r,x); for(int i=l;i<r;++i) a[i]=std::min(a[i],x); }
            else if(op==1) { tree.range_chmax(l,r,x); for(int i=l;i<r;++i) a[i]=std::max(a[i],x); }
            else if(op==2) { tree.range_add(l,r,x); for(int i=l;i<r;++i) a[i]+=x; }
            long long sum=0,lo=tree.INF,hi=-tree.INF;
            for(int i=l;i<r;++i) { sum+=a[i]; lo=std::min(lo,a[i]); hi=std::max(hi,a[i]); }
            require(tree.range_sum(l,r)==sum && tree.range_min(l,r)==lo && tree.range_max(l,r)==hi,"Beats vs direct array");
        }
        for(int i=0;i<n;++i) require(tree.get(i)==a[i],"Beats all leaves");
    }
    cp::SegmentTreeBeats large(std::vector<long long>(200000,100)); large.range_chmin(0,200000,-100); large.range_add(123,199999,7); large.range_chmax(500,190000,0);
    require(large.range_sum(500,190000)==0,"Beats large range");
    const auto inf=cp::SegmentTreeBeats::INF;
    cp::SegmentTreeBeats hidden_hi(std::vector<long long>{inf-1,0});
    hidden_hi.range_chmin(0,2,1); hidden_hi.range_add(0,2,inf-2);
    require(hidden_hi.get(0)==inf-1 && hidden_hi.get(1)==inf-2,"Beats cap before add with stale large child");
    cp::SegmentTreeBeats hidden_lo(std::vector<long long>{-inf+1,0});
    hidden_lo.range_chmax(0,2,-1); hidden_lo.range_add(0,2,-inf+2);
    require(hidden_lo.get(0)==-inf+1 && hidden_lo.get(1)==-inf+2,"Beats floor before add with stale small child");
}
void static_order() {
    for(int trial=0;trial<150;++trial) {
        int n=pick(100); std::vector<long long> a(n); for(auto &x:a) x=pick(31)-15;
        if(n>3) { a[0]=std::numeric_limits<long long>::min(); a[1]=std::numeric_limits<long long>::max(); }
        cp::WaveletMatrix<long long> wm(a); cp::PersistentOrderStatistics<long long> ps(a);
        for(int i=0;i<n;++i) require(wm.access(i)==a[i],"wavelet access");
        for(int q=0;q<250;++q) {
            int l=pick(n+1),r=pick(n+1); if(l>r) std::swap(l,r); std::vector<long long> sorted(a.begin()+l,a.begin()+r); std::sort(sorted.begin(),sorted.end());
            int k=pick(r-l+3)-1; auto x=pick(41)-20; int less=int(std::lower_bound(sorted.begin(),sorted.end(),x)-sorted.begin());
            std::optional<long long> want; if(0<=k && k<r-l) want=sorted[k];
            require(wm.kth_smallest(l,r,k)==want && ps.kth_smallest(l,r,k)==want,"both static kth vs sorting");
            require(wm.count_less(l,r,x)==less && ps.count_less(l,r,x)==less,"static counts");
            require(wm.count(l,r,x)==std::count(sorted.begin(),sorted.end(),x),"wavelet exact count");
            require(wm.range_freq(l,r,x,x+7)==std::count_if(sorted.begin(),sorted.end(),[&](long long v){return x<=v && v<x+7;}) && wm.range_freq(l,r,x,x+7)==ps.range_freq(l,r,x,x+7),"range frequency");
            std::optional<long long> pred,succ; if(less) pred=sorted[less-1]; if(less<r-l) succ=sorted[less];
            require(wm.predecessor(l,r,x)==pred && wm.successor(l,r,x)==succ,"wavelet neighbors");
        }
    }
    std::vector<int> a(200000); std::iota(a.rbegin(),a.rend(),0); cp::WaveletMatrix<int> w(a); require(w.kth_smallest(0,200000,100000)==100000,"large wavelet");
}
void persistent() {
    for(int n:{1,2,17,70}) {
        cp::PersistentSegmentTree<std::string,cat,empty_string> tree(n); std::vector<int> roots{0}; std::vector<std::vector<std::string>> histories(1,std::vector<std::string>(n));
        for(int q=0;q<1500;++q) {
            int version=pick(int(roots.size())),p=pick(n); std::string x(1,char('a'+pick(26))); auto next=histories[version]; next[p]=x;
            roots.push_back(tree.set(roots[version],p,x)); histories.push_back(next);
            version=pick(int(roots.size())); int l=pick(n+1),r=pick(n+1); if(l>r) std::swap(l,r); std::string want;
            for(int i=l;i<r;++i) want+=histories[version][i];
            require(tree.prod(roots[version],l,r)==want,"branching persistent noncommutative histories");
        }
    }
    cp::PersistentSegmentTree<long long,sum_op,zero> empty(0); require(empty.prod(0,0,0)==0,"empty persistent");
}
void treap() {
    cp::ImplicitTreap<std::string,cat,empty_string> t; std::vector<std::string> a;
    for(int q=0;q<30000;++q) {
        int n=int(a.size()),op=pick(8),l=pick(n+1),r=pick(n+1); if(l>r) std::swap(l,r);
        if(op==0 || n==0) { std::string x(1,char('a'+pick(26))); t.insert(l,x); a.insert(a.begin()+l,x); }
        else if(op==1) { int p=pick(n); t.erase(p); a.erase(a.begin()+p); }
        else if(op==2) { t.reverse(l,r); std::reverse(a.begin()+l,a.begin()+r); }
        else if(op==3) { int p=pick(n); t.set(p,"XYZ"); a[p]="XYZ"; }
        else if(op==4) { int p=pick(n-(r-l)+1); auto part=std::vector<std::string>(a.begin()+l,a.begin()+r); a.erase(a.begin()+l,a.begin()+r); a.insert(a.begin()+p,part.begin(),part.end()); t.move(l,r,p); }
        else { std::string want; for(int i=l;i<r;++i) want+=a[i]; require(t.prod(l,r)==want,"treap noncommutative prod"); }
        require(t.size()==int(a.size()) && t.to_vector()==a,"treap vector operations");
        std::string all; for(const auto &x:a) all+=x; require(t.all_prod()==all,"treap full product");
        if(q%1000==0) { auto copy=t; copy.insert(0,"!"); require(copy.size()==t.size()+1,"treap independent copy"); }
    }
    t.clear(); require(t.empty() && t.all_prod().empty(),"treap clear");
    cp::ImplicitTreap<NonDefault,combine,nd_zero> custom;
    custom.insert(0,NonDefault(5)); custom.clear(); custom.insert(0,NonDefault(7));
    require(custom.all_prod().value==7,"Treap e() without a default constructor");
}
bool connected(int n,const std::map<std::pair<int,int>,int>& edges,int u,int v) {
    std::vector<bool> seen(n); std::vector<int> q{u}; seen[u]=true;
    for(std::size_t i=0;i<q.size();++i) for(auto [e,count]:edges) if(count) { int next=e.first==q[i]?e.second:e.second==q[i]?e.first:-1; if(next!=-1 && !seen[next]) { seen[next]=true; q.push_back(next); } }
    return seen[v];
}
void connectivity() {
    cp::RollbackDisjointSetUnion dsu(8); auto state=dsu.snapshot(); require(dsu.merge(0,1) && !dsu.merge(1,0),"rollback failed merge event"); dsu.undo(); require(dsu.same(0,1),"undo failed merge"); auto later=dsu.snapshot(); dsu.merge(1,2); require(dsu.component_size(0)==3,"rollback size"); dsu.rollback(later); require(!dsu.same(0,2),"rollback branch"); dsu.rollback(state); require(dsu.component_count()==8,"rollback component count");
    for(int trial=0;trial<120;++trial) {
        int n=1+pick(15); cp::OfflineDynamicConnectivity c(n); std::map<std::pair<int,int>,int> edges; std::vector<bool> want;
        for(int q=0;q<400;++q) {
            int u=pick(n),v=pick(n),op=pick(3); auto key=std::minmax(u,v); std::pair<int,int> e=key;
            if(op==0) { c.add_edge(u,v); ++edges[e]; }
            else if(op==1) { bool exists=edges[e]>0; require(c.remove_edge(u,v)==exists,"remove missing edge"); if(exists) --edges[e]; }
            else { require(c.add_query(u,v)==int(want.size()),"connectivity query id"); want.push_back(connected(n,edges,u,v)); }
            if(q%100==0) require(c.solve()==want,"connectivity prefix solve");
        }
        require(c.solve()==want && c.solve()==want,"connectivity repeated solve vs BFS");
    }
}
void mo_updates() {
    struct Update { int p,before,after; };
    for(int trial=0;trial<100;++trial) {
        int n=1+pick(40); cp::MoWithUpdates mo(n); std::vector<int> initial(n); for(int &x:initial) x=pick(12);
        auto a=initial; std::vector<Update> updates; std::vector<int> want;
        for(int i=0;i<300;++i) {
            if(pick(2)) { int p=pick(n),x=pick(12); require(mo.add_update()==int(updates.size()),"Mo update id"); updates.push_back({p,a[p],x}); a[p]=x; }
            else { int l=pick(n+1),r=pick(n+1); if(l>r) std::swap(l,r); std::vector<int> seen(12); for(int j=l;j<r;++j) seen[a[j]]=1; want.push_back(std::accumulate(seen.begin(),seen.end(),0)); mo.add_query(l,r); }
        }
        for(int block:{0,1,7}) {
            a=initial; std::vector<int> count(12),answer(want.size()); int distinct=0;
            auto add=[&](int p){ if(count[a[p]]++==0) ++distinct; };
            auto remove=[&](int p){ if(--count[a[p]]==0) --distinct; };
            auto change=[&](int id,int l,int r,bool forward) { auto u=updates[id]; if(l<=u.p && u.p<r) remove(u.p); a[u.p]=forward?u.after:u.before; if(l<=u.p && u.p<r) add(u.p); };
            mo.run(add,remove,[&](int id,int l,int r){change(id,l,r,true);},[&](int id,int l,int r){change(id,l,r,false);},[&](int id){answer[id]=distinct;},block);
            require(answer==want,"Mo with updates distinct vs snapshots");
        }
    }
    cp::MoWithUpdates empty(0); empty.add_query(0,0); int calls=0; empty.run([](int){},[](int){},[](int,int,int){},[](int,int,int){},[&](int){++calls;}); require(calls==1,"empty Mo");
}
int main() { beats(); static_order(); persistent(); treap(); connectivity(); mo_updates(); std::cout<<"structures: randomized arrays, histories, multigraph BFS, and update-time Mo passed\n"; }
