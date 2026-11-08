#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numeric>
#include <queue>
#include <random>
#include <stdexcept>
#include <vector>
#include "basic_utility/rollback_array.hpp"
#include "union_find/disjoint_set_union.hpp"
#include "union_find/rollback_disjoint_set_union_with_data.hpp"
#include "offline_query/segment_tree_over_time.hpp"
#include "offline_query/offline_dynamic_connectivity.hpp"
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << "\n"; std::abort(); } } while (false)
using Data=cp::RollbackArray<long long>;
using DSU=cp::RollbackDisjointSetUnionWithData<Data>;
auto combine=[](auto& data,int kept,int absorbed){ data.set(kept,data[kept]+data[absorbed]); };
int main() {
    cp::DisjointSetUnion ordinary(4); int callbacks=0;
    CHECK(ordinary.merge(1,2,[&](int a,int b){CHECK(a==1 && b==2); CHECK(!ordinary.same(a,b)); ++callbacks;}));
    CHECK(ordinary.merge(0,2,[&](int a,int b){CHECK(a==1 && b==0); ++callbacks;}));
    CHECK(!ordinary.merge(0,1,[&](int,int){++callbacks;})); CHECK(callbacks==2);
    try { ordinary.merge(0,3,[](int,int){throw std::runtime_error("callback");}); CHECK(false); } catch (const std::runtime_error&) {}
    CHECK(ordinary.component_count()==2 && !ordinary.same(0,3));
    std::mt19937 rng(12365);
    for (int repetition=0;repetition<80;++repetition) {
        int n=1+rng()%30; std::vector<long long> weights(n); for (auto& x:weights) x=rng()%21;
        Data data(weights); DSU dsu(n,data); std::vector<int> groups(n); std::iota(groups.begin(),groups.end(),0);
        struct Saved { DSU::Snapshot token; std::vector<int> groups; std::vector<long long> values; };
        std::vector<Saved> saved{{dsu.snapshot(),groups,data.values()}};
        for (int t=0;t<1000;++t) {
            if (rng()%4==0) {
                std::size_t i=rng()%saved.size(); auto state=saved[i]; dsu.rollback(state.token); groups=state.groups;
                CHECK(data.values()==state.values); saved.erase(saved.begin()+i+1,saved.end());
            } else {
                int a=rng()%n,b=rng()%n; bool want=groups[a]!=groups[b]; int before=groups[b],after=groups[a];
                CHECK(dsu.merge(a,b,combine)==want);
                for (int& x:groups) if (x==before) x=after;
                saved.push_back({dsu.snapshot(),groups,data.values()});
            }
            for (int a=0;a<n;++a) {
                int count=0; long long sum=0;
                for (int b=0;b<n;++b) { CHECK(dsu.same(a,b)==(groups[a]==groups[b])); if (groups[a]==groups[b]) {++count;sum+=weights[b];} }
                CHECK(dsu.component_size(a)==count && data[dsu.leader(a)]==sum);
            }
        }
    }
    Data data(std::vector<long long>{1,2,3,0}); DSU dsu(3,data); auto first=dsu.snapshot();
    dsu.merge(0,1,combine); auto stale=dsu.snapshot(); dsu.rollback(first); dsu.merge(1,2,combine);
    try { dsu.rollback(stale); CHECK(false); } catch (const std::invalid_argument&) {}
    auto saved=dsu.snapshot(); auto values=data.values();
    try { dsu.merge(0,1,[](auto& a,int,int){a.set(3,99);throw std::runtime_error("partial metadata");}); CHECK(false); } catch (const std::runtime_error&) {}
    CHECK(data.values()==values && dsu.component_count()==2); dsu.rollback(saved);
    data.set(3,7); saved=dsu.snapshot(); dsu.merge(1,2,combine); data.set(3,8); dsu.undo(); CHECK(data[3]==7); dsu.rollback(saved);
    Data unrelated(std::vector<long long>{1,2,3}); DSU other(3,unrelated);
    try { other.rollback(first); CHECK(false); } catch (const std::invalid_argument&) {}
    dsu.rollback(first); CHECK(data.values()==std::vector<long long>({1,2,3,0}));
    for (int times=0;times<=100;++times) {
        cp::SegmentTreeOverTime<std::pair<int,int>> timeline(times);
        std::vector<std::vector<std::pair<int,int>>> edges(times);
        for (int t=0;t<100;++t) {
            int l=rng()%(times+1),r=rng()%(times+1); if (l>r) std::swap(l,r);
            int a=rng()%8,b=rng()%8; timeline.add(l,r,{a,b});
            for (int i=l;i<r;++i) edges[i].emplace_back(a,b);
        }
        Data sums(std::vector<long long>{1,2,3,4,5,6,7,8}); DSU state(8,sums);
        auto apply=[](auto& s,const auto& edge){s.merge(edge.first,edge.second,combine);};
        for (int repeat=0;repeat<2;++repeat) {
            int visits=0;
            timeline.run(state,apply,[&](auto& s,int time){
                CHECK(time==visits++);
                std::vector<std::vector<int>> adj(8);
                for (auto [a,b]:edges[time]) {adj[a].push_back(b);adj[b].push_back(a);}
                for (int start=0;start<8;++start) {
                    std::vector<int> seen(8); std::queue<int> q; q.push(start); seen[start]=1; long long total=0;
                    while (!q.empty()) {int v=q.front();q.pop();total+=v+1;for (int u:adj[v]) if (!seen[u]) {seen[u]=1;q.push(u);}}
                    for (int v=0;v<8;++v) CHECK(s.same(start,v)==bool(seen[v]));
                    CHECK(s.data()[s.leader(start)]==total);
                }
            });
            CHECK(visits==times && state.component_count()==8 && sums.history_size()==0);
        }
        if (times) {
            try { timeline.run(state,apply,[](auto&,int){throw std::runtime_error("visit");}); CHECK(false); } catch (const std::runtime_error&) {}
            CHECK(state.component_count()==8 && sums.history_size()==0);
            timeline.add(0,times,{0,1});
            try { timeline.run(state,[](auto& s,const auto&){s.data().set(0,99);throw std::runtime_error("apply");},[](auto&,int){}); CHECK(false); } catch (const std::runtime_error&) {}
            CHECK(state.component_count()==8 && sums[0]==1 && sums.history_size()==0);
        }
    }
    struct SumState {
        long long value=0;
        long long snapshot() const {return value;}
        void rollback(long long x) {value=x;}
        void apply(const std::unique_ptr<int>& x) {value+=*x;}
    } state;
    state.value=11;
    cp::SegmentTreeOverTime<std::unique_ptr<int>> moving(3); moving.add(0,2,std::make_unique<int>(7));
    moving.run(state,[](auto& s,int t){CHECK(s.value==11+(t<2?7:0));}); CHECK(state.value==11);
    cp::OfflineDynamicConnectivity dc(3); dc.add_edge(0,1); dc.add_edge(0,1); dc.remove_edge(0,1); dc.add_query(0,1);
    dc.remove_edge(0,1); dc.add_query(0,1); CHECK(!dc.remove_edge(0,1)); CHECK(dc.solve()==std::vector<bool>({true,false})); CHECK(dc.solve()==dc.solve());
    std::cout << "rollback tests: 80000 branching operations, callbacks, stale/foreign snapshots, arbitrary metadata, 101 timelines against BFS, repeat/exception restoration and move-only events passed\n";
}
