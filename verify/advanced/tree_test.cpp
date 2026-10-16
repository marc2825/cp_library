#include <algorithm>
#include <iostream>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <vector>
#include "common.hpp"
#include "tree/all.hpp"
std::string concat(std::string a,std::string b) { return a+b; }
std::string identity() { return {}; }
long long sum(long long a,long long b) { return a+b; }
long long zero() { return 0; }
std::mt19937 rng(3112026);
int pick(int n) { return int(rng()%n); }
std::vector<int> path(const std::vector<std::set<int>>& adj,int s,int t) {
    std::vector<int> parent(adj.size(),-1),queue{s}; parent[s]=s;
    for(std::size_t i=0;i<queue.size();++i) for(int to:adj[queue[i]]) if(parent[to]==-1) { parent[to]=queue[i]; queue.push_back(to); }
    if(parent[t]==-1) return {};
    std::vector<int> result; for(int v=t;;v=parent[v]) { result.push_back(v); if(v==s) break; } std::reverse(result.begin(),result.end()); return result;
}
void link_cut() {
    for(int trial=0;trial<90;++trial) {
        int n=1+pick(40); std::vector<std::string> values(n); for(auto &x:values) x=std::string(1,char('a'+pick(26)));
        cp::LinkCutTree<std::string,concat,identity> tree(values); std::vector<std::set<int>> adj(n);
        for(int q=0;q<1500;++q) {
            int u=pick(n),v=pick(n),op=pick(6); auto vertices=path(adj,u,v);
            if(op==0) { bool ok=vertices.empty(); require(tree.link(u,v)==ok,"LCT link cycle detection"); if(ok) { adj[u].insert(v); adj[v].insert(u); } }
            else if(op==1) { bool ok=adj[u].count(v); require(tree.cut(u,v)==ok,"LCT cut direct edges only"); if(ok) { adj[u].erase(v); adj[v].erase(u); } }
            else if(op==2) { values[u]=std::string(1,char('A'+pick(26))); tree.set(u,values[u]); }
            else if(op==3) tree.make_root(u);
            else {
                require(tree.connected(u,v)==!vertices.empty(),"LCT connectivity BFS"); std::optional<std::string> expected;
                if(!vertices.empty()) { expected=""; for(int x:vertices) *expected+=values[x]; }
                require(tree.path_prod(u,v)==expected,"LCT directed noncommutative path product");
                int k=pick(n+2)-1; int want=0<=k && k<int(vertices.size())?vertices[k]:-1;
                require(tree.kth_on_path(u,v,k)==want,"LCT kth path vertex");
            }
            require(tree.get(u)==values[u],"LCT vertex get after reroot");
        }
    }
    int n=200000; cp::LinkCutTree<long long,sum,zero> tree(std::vector<long long>(n,1));
    for(int i=1;i<n;++i) require(tree.link(i-1,i),"LCT deep chain link");
    require(tree.path_prod(0,n-1)==n && tree.kth_on_path(n-1,0,n/2)==n-1-n/2,"LCT deep chain no recursion");
    require(tree.cut(n/2-1,n/2) && !tree.connected(0,n-1),"LCT deep chain cut");
}
void static_tree() {
    for(int trial=0;trial<180;++trial) {
        int n=pick(70); cp::Graph<long long> g(n); std::vector<std::set<int>> adj(n);
        for(int v=1;v<n;++v) if(pick(5)) { int p=pick(v); g.add_edge(p,v); adj[p].insert(v); adj[v].insert(p); }
        cp::VirtualTree<long long> vt(g); const auto &tree=vt.lca.tree;
        for(int q=0;q<50;++q) {
            std::vector<int> important; for(int v=0;v<n;++v) if(!pick(4)) { important.push_back(v); if(!pick(3)) important.push_back(v); }
            auto result=vt.build(important); std::set<int> closure(important.begin(),important.end());
            for(int a:important) for(int b:important) { int w=vt.lca.lca(a,b); if(w!=-1) closure.insert(w); }
            require(std::set<int>(result.vertices.begin(),result.vertices.end())==closure,"virtual tree all-pairs LCA closure");
            std::set<int> marked(important.begin(),important.end());
            for(int i=0;i<int(result.vertices.size());++i) {
                int v=result.vertices[i],p=-1;
                for(int candidate:closure) if(candidate!=v && tree.is_ancestor(candidate,v) && (p==-1 || tree.depth[candidate]>tree.depth[p])) p=candidate;
                require((result.parent[i]==-1?-1:result.vertices[result.parent[i]])==p,"virtual tree nearest retained ancestor");
                require(result.marked[i]==bool(marked.count(v)),"virtual tree marks");
                require(result.edge_length[i]==(p==-1?0:tree.depth[v]-tree.depth[p]),"virtual edge length");
            }
        }
        std::vector<int> colors(n); for(int &x:colors) x=pick(11);
        std::vector<int> count(11),answer(n); int distinct=0;
        cp::dsu_on_tree(tree,[&](int v){if(count[colors[v]]++==0)++distinct;},[&](int v){if(--count[colors[v]]==0)--distinct;},[&](int v){answer[v]=distinct;});
        require(distinct==0 && std::accumulate(count.begin(),count.end(),0)==0,"sack clears every forest root");
        for(int v=0;v<n;++v) { std::set<int> bag; for(int w=0;w<n;++w) if(tree.is_ancestor(v,w)) bag.insert(colors[w]); require(answer[v]==int(bag.size()),"sack subtree distinct vs direct ancestors"); }
    }
    int n=200000; cp::Graph<int> g(n); for(int i=1;i<n;++i) g.add_edge(i-1,i);
    cp::RootedTree<int> tree(g); int size=0,answers=0;
    cp::dsu_on_tree(tree,[&](int){++size;},[&](int){--size;},[&](int v){require(size==n-v,"deep sack subtree");++answers;}); require(size==0 && answers==n,"nonrecursive sack");
    cp::VirtualTree<int> vt(g); auto result=vt.build({n-1,n/2,n-1}); require(result.vertices==std::vector<int>({n/2,n-1}) && result.parent==std::vector<int>({-1,0}),"deep virtual tree duplicates");
}
int main() { link_cut(); static_tree(); std::cout<<"trees: dynamic forest BFS paths, LCA closure, subtree sets and 200000-node chains passed\n"; }
