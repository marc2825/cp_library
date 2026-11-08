#include <algorithm>
#include <iostream>
#include <numeric>
#include <queue>
#include <random>
#include <stdexcept>
#include <vector>
#include "union_find/all.hpp"
void check(bool ok) { if(!ok) throw std::runtime_error("DSU differential mismatch"); }
long long add(long long a,long long b) {return a+b;} long long zero(){return 0;}
void join(std::vector<int>& a,int u,int v) { int x=a[v],y=a[u]; for(int& z:a) if(z==x) z=y; }
int main() {
    std::mt19937 rng(98321); constexpr int n=25;
    for(int trial=0;trial<60;++trial) {
        cp::PartiallyPersistentDisjointSetUnion partial(n); cp::PersistentDisjointSetUnion persistent(n);
        cp::AggregateDisjointSetUnion<long long,add,zero> aggregate(std::vector<long long>(n,1));
        cp::MergeTree tree(n); cp::IntervalDisjointSetUnion intervals(n);
        std::vector<int> init(n); std::iota(init.begin(),init.end(),0);
        std::vector<std::vector<int>> past{init},branches{init}; std::vector<int> roots{0};
        auto current=init,interval_labels=init; std::vector<long long> weights(n,1);
        for(int step=0;step<100;++step) {
            int u=rng()%n,v=rng()%n,t=rng()%past.size();
            bool changed=current[u]!=current[v]; check(partial.merge(u,v)==changed);
            check(aggregate.merge(u,v)==changed); tree.merge(u,v); join(current,u,v); past.push_back(current);
            int base=rng()%branches.size(); auto labels=branches[base]; join(labels,u,v);
            roots.push_back(persistent.merge(roots[base],u,v)); branches.push_back(labels);
            check(partial.time()==step+1);
            int l=rng()%(n+1),r=rng()%(n+1); if(l>r) std::swap(l,r);
            intervals.merge(l,r); for(int i=l+1;i<r;++i) join(interval_labels,l,i);
            int x=rng()%n; long long w=int(rng()%11)-5; aggregate.add(x,w); weights[x]+=w;
            for(int a=0;a<n;++a) {
                int sz=0,old_sz=0,branch_sz=0; long long sum=0;
                for(int b=0;b<n;++b) {
                    sz+=current[a]==current[b]; old_sz+=past[t][a]==past[t][b]; branch_sz+=labels[a]==labels[b];
                    if(current[a]==current[b]) sum+=weights[b];
                    check(partial.same(t,a,b)==(past[t][a]==past[t][b]));
                    check(persistent.same(roots.back(),a,b)==(labels[a]==labels[b]));
                    check(intervals.same(a,b)==(interval_labels[a]==interval_labels[b]));
                    check((tree.component_root(a)==tree.component_root(b))==(current[a]==current[b]));
                }
                check(partial.size(t,a)==old_sz); check(persistent.size(roots.back(),a)==branch_sz);
                check(aggregate.size(a)==sz && aggregate.prod(a)==sum);
            }
            for(int k=n;k<int(tree.children.size());++k) {
                auto c=tree.children[k]; check(c[0]<k && c[1]<k); check(tree.parent[c[0]]==k && tree.parent[c[1]]==k);
            }
        }
        // BFSによる各成分の二部性を辺追加のたびに確認。
        cp::BipartiteDisjointSetUnion bip(n); std::vector<std::vector<int>> g(n);
        for(int step=0;step<100;++step) {
            int u=rng()%n,v=rng()%n; g[u].push_back(v);g[v].push_back(u); bip.add_edge(u,v);
            std::vector<int> color(n,-1); bool global=true;
            for(int root=0;root<n;++root) if(color[root]==-1) {
                std::queue<int> q; q.push(root); color[root]=0; std::vector<int> group; bool ok=true;
                while(!q.empty()) {int a=q.front();q.pop();group.push_back(a);for(int b:g[a]) {if(color[b]==-1){color[b]=color[a]^1;q.push(b);}else if(color[b]==color[a])ok=false;}}
                for(int a:group) check(bip.component_is_bipartite(a)==ok);
                global=global&&ok;
            }
            check(bip.is_bipartite()==global);
        }
    }
    cp::IntervalDisjointSetUnion large(200000); large.merge(0,200000); large.merge(0,200000); check(large.size(100)==200000);
    cp::MergeTree empty(0); check(empty.roots().empty());
    std::cout<<"DSU: 60 histories, all-pair past/branch checks, aggregates, merge forests, parity BFS and 200000 elements passed\n";
}
