#include <algorithm>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include "tree/tree_hash.hpp"
void check(bool ok){if(!ok)throw std::runtime_error("tree hash mismatch");}
std::string canonical(const cp::Graph<int>&g,int v,int parent=-1){std::vector<std::string>children;for(auto e:g[v])if(e.to!=parent)children.push_back(canonical(g,e.to,v));std::sort(children.begin(),children.end());std::string s="(";for(auto c:children)s+=c;return s+")";}
int main(){
    std::mt19937 rng(987123);std::map<std::string,cp::TreeHash>rooted,unrooted;std::map<cp::TreeHash,std::string>reverse;
    int cases=0;
    for(int n=1;n<=6;++n){int count=1;for(int i=0;i<n-2;++i)count*=n;for(int code=0;code<count;++code){
        cp::Graph<int>g(n);std::vector<int>sequence(std::max(0,n-2)),degree(n,1);int c=code;for(int& x:sequence){x=c%n;c/=n;++degree[x];}for(int v:sequence){int leaf=0;while(degree[leaf]!=1)++leaf;g.add_edge(leaf,v);--degree[leaf];--degree[v];}if(n>1){int a=0;while(degree[a]!=1)++a;int b=a+1;while(degree[b]!=1)++b;g.add_edge(a,b);}
        std::string canonical_unrooted;
        for(int root=0;root<n;++root){auto key=canonical(g,root); auto h=cp::rooted_tree_hash(g,root).hash[root];if(!rooted.count(key))rooted[key]=h;check(rooted[key]==h);if(!reverse.count(h))reverse[h]=key;check(reverse[h]==key);if(canonical_unrooted.empty()||key<canonical_unrooted)canonical_unrooted=key;}
        auto h=cp::unrooted_tree_hash(g);if(!unrooted.count(canonical_unrooted))unrooted[canonical_unrooted]=h;check(unrooted[canonical_unrooted]==h);
        std::vector<int>p(n);std::iota(p.begin(),p.end(),0);std::shuffle(p.begin(),p.end(),rng);cp::Graph<int>renamed(n);auto edges=g.edges();std::shuffle(edges.begin(),edges.end(),rng);for(auto e:edges)renamed.add_edge(p[e.from],p[e.to]);check(cp::unrooted_tree_hash(renamed)==h);++cases;
    }}
    cp::Graph<int>empty(0);check(cp::unrooted_tree_hash(empty).size==0);cp::Graph<int>forest(2);check(cp::rooted_tree_hash(forest).roots.size()==2);bool thrown=false;try{cp::unrooted_tree_hash(forest);}catch(const std::invalid_argument&){thrown=true;}check(thrown);
    cp::Graph<int>bad(2);bad.add_edge(0,1);bad.add_edge(0,1);thrown=false;try{cp::rooted_tree_hash(bad);}catch(const std::invalid_argument&){thrown=true;}check(thrown);
    cp::Graph<int>large(200000);for(int i=1;i<large.size();++i)large.add_edge(i-1,i);check(cp::unrooted_tree_hash(large).size==200000);
    std::cout<<"tree hash: "<<cases<<" labeled trees through n=6 against canonical strings, relabeling, invalid forests and 200000 vertices passed\n";
}
