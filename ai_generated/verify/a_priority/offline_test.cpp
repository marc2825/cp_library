#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>
#include "offline_query/all.hpp"
void check(bool ok){if(!ok)throw std::runtime_error("offline differential mismatch");}
int main(){
    std::mt19937 rng(1987123);
    for(int n=0;n<=100;++n){
        std::vector<int>a(n);for(int& x:a)x=rng()%20;
        std::vector<std::pair<int,int>> queries;cp::RollbackMo mo(n);
        for(int q=0;q<200;++q){int l=rng()%(n+1),r=rng()%(n+1);if(l>r)std::swap(l,r);queries.emplace_back(l,r);mo.add_query(l,r);}
        auto rmq=cp::offline_rmq(a,queries);auto maxq=cp::offline_rmq(a,queries,std::greater<int>{});
        for(int b:{1,7,150}){
            std::vector<int>state;std::vector<long long>answers(queries.size());long long sum=0;
            auto reset=[&]{state.clear();sum=0;};auto add=[&](int p){state.push_back(p);sum+=a[p];};
            mo.run(reset,add,add,[&]{return state.size();},[&](std::size_t size){while(state.size()>size){sum-=a[state.back()];state.pop_back();}},[&](int id){answers[id]=sum;},b);
            check(state.empty());
            for(int q=0;q<int(queries.size());++q){auto [l,r]=queries[q];check(answers[q]==std::accumulate(a.begin()+l,a.begin()+r,0LL));int lo=l==r?-1:int(std::min_element(a.begin()+l,a.begin()+r)-a.begin());int hi=l==r?-1:int(std::max_element(a.begin()+l,a.begin()+r)-a.begin());check(rmq[q]==lo&&maxq[q]==hi);}
        }
    }
    cp::RangeSet<int> ranges;std::set<int> points;
    for(int step=0;step<10000;++step){
        int l=int(rng()%101)-50,r=int(rng()%101)-50;if(l>r)std::swap(l,r);
        if(rng()%2){ranges.insert(l,r);for(int i=l;i<r;++i)points.insert(i);}else{ranges.erase(l,r);for(int i=l;i<r;++i)points.erase(i);}
        for(int i=-51;i<=51;++i)check(ranges.contains(i)==bool(points.count(i)));
        bool covered=true;for(int i=l;i<r;++i)covered=covered&&points.count(i);check(ranges.covers(l,r)==covered);
        std::vector<std::pair<int,int>> expected;for(int x:points){if(expected.empty()||expected.back().second!=x)expected.emplace_back(x,x+1);else++expected.back().second;}check(ranges.intervals()==expected);
    }
    for(int trial=0;trial<150;++trial){
        int n=1+rng()%50;cp::Graph<int> g(n);std::vector<int> parent(n,-1);
        for(int v=1;v<n;++v){parent[v]=rng()%v;g.add_edge(v,parent[v]);}
        cp::MoOnTree mo(g,int(rng()%n));std::vector<std::pair<int,int>> queries;
        std::vector<std::set<int>> expected;
        for(int q=0;q<200;++q){
            int u=rng()%n,v=rng()%n;queries.emplace_back(u,v);mo.add_query(u,v);
            // 独立BFSで経路を復元。
            std::vector<int> prev(n,-1),queue{u};prev[u]=u;
            for(std::size_t p=0;p<queue.size();++p)for(auto e:g[queue[p]])if(prev[e.to]==-1){prev[e.to]=queue[p];queue.push_back(e.to);}
            std::set<int> path;for(int x=v;;x=prev[x]){path.insert(x);if(x==u)break;}expected.push_back(path);
        }
        for(int block:{0,1,9}){
            std::set<int> active;
            mo.run([&](int v){check(active.insert(v).second);},[&](int v){check(active.erase(v)==1);},[&](int id){check(active==expected[id]);},block);
        }
    }
    cp::Graph<int> forest(3);forest.add_edge(0,1);cp::MoOnTree mf(forest);mf.add_query(2,2);int sum=0;mf.run([&](int v){sum+=v;},[&](int v){sum-=v;},[&](int){check(sum==2);});
    bool thrown=false;try{mf.add_query(0,2);}catch(const std::invalid_argument&){thrown=true;}check(thrown);
    cp::Graph<int> empty(0);cp::MoOnTree none(empty);none.run([](int){},[](int){},[](int){});
    cp::Graph<int> large(200000);for(int i=1;i<200000;++i)large.add_edge(i-1,i);cp::MoOnTree path(large);path.add_query(0,199999);int count=0;path.run([&](int){++count;},[&](int){--count;},[&](int){check(count==200000);});
    std::cout<<"offline: RMQ/Rollback Mo at 3 block widths, 10000 interval edits, 30000 BFS paths and 200000-vertex path passed\n";
}
