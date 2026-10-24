#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <queue>
#include <random>
#include <utility>
#include <vector>
#include "../../tree/dynamic_subtree_aggregate.hpp"
#include "../../tree/fully_dynamic_tree_dp.hpp"
#include "../../range_query/range_actions.hpp"
#include "../../basic_utility/static_modint.hpp"
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << '\n'; std::abort(); } } while (false)
struct Forest {
    int n;
    std::vector<std::vector<int>> edge;
    explicit Forest(int n):n(n),edge(n,std::vector<int>(n)){}
    std::vector<int> parents(int root) const {
        std::vector<int> p(n,-2), q{root}; p[root]=-1;
        for (std::size_t i=0;i<q.size();++i) for(int v=0;v<n;++v)
            if(edge[q[i]][v] && p[v]==-2) {p[v]=q[i];q.push_back(v);}
        return p;
    }
    std::vector<int> path(int u,int v) const {
        auto p=parents(u); std::vector<int> a;
        if(p[v]==-2)return a;
        for(;v!=-1;v=p[v])a.push_back(v);
        std::reverse(a.begin(),a.end());return a;
    }
    std::vector<int> subtree(int r,int v)const {
        auto p=parents(r);std::vector<int>a;
        if(p[v]==-2)return a;
        for(int u=0;u<n;++u)if(p[u]!=-2){int x=u;while(x!=-1 && x!=v)x=p[x];if(x==v)a.push_back(u);}
        return a;
    }
    bool link(int u,int v){if(!path(u,v).empty())return false;edge[u][v]=edge[v][u]=1;return true;}
    bool cut(int u,int v){if(!edge[u][v])return false;edge[u][v]=edge[v][u]=0;return true;}
};
using A=cp::AffineSumAction<long long>;
using Tree=cp::DynamicSubtreeAggregate<A>;
void lazy_random(){
    std::mt19937 rng(41231);
    for(int seed=0;seed<100;++seed){
        int n=1+int(rng()%24);Forest f(n);std::vector<long long>a(n);std::iota(a.begin(),a.end(),-10);
        Tree tree(A::values(a));
        for(int v=1;v<n;++v){int u=rng()%v;CHECK(tree.link(u,v));CHECK(f.link(u,v));}
        for(int step=0;step<1500;++step){
            int u=rng()%n,v=rng()%n,mode=rng()%12;
            if(mode==0)CHECK(tree.link(u,v)==f.link(u,v));
            else if(mode==1)CHECK(tree.cut(u,v)==f.cut(u,v));
            else if(mode==2){long long x=int(rng()%31)-15;tree.set(u,{x,1});a[u]=x;}
            else if(mode<=5){
                A::F tag{int(rng()%3)-1,int(rng()%9)-4};
                auto target=mode==3?f.path(u,v):f.subtree(u,mode==4?v:u);
                if(mode==3)CHECK(tree.path_apply(u,v,tag)==!target.empty());
                else if(mode==4)CHECK(tree.subtree_apply(u,v,tag)==!target.empty());
                else tree.component_apply(u,tag);
                for(int x:target)a[x]=tag.a*a[x]+tag.b;
            }else if(mode==6)tree.make_root(u);
            else if(mode==7)CHECK(tree.get(u).sum==a[u]);
            else{
                auto target=mode==8?f.path(u,v):f.subtree(u,mode==9?v:u);
                auto got=mode==8?tree.path_prod(u,v):mode==9?tree.subtree_prod(u,v):std::optional<A::S>(tree.component_prod(u));
                CHECK(bool(got)==!target.empty());
                if(got){long long sum=0;for(int x:target)sum+=a[x];CHECK(got->sum==sum);CHECK(got->length==int(target.size()));}
            }
            if(step%47==0)for(int x=0;x<n;++x)CHECK(tree.get(x).sum==a[x]);
            CHECK(tree.allocated_nodes()<=std::size_t(2*n));
        }
        Tree moved(std::move(tree));for(int v=0;v<n;++v)CHECK(moved.get(v).sum==a[v]);
        Tree assigned;assigned=std::move(moved);for(int v=0;v<n;++v)CHECK(assigned.get(v).sum==a[v]);
    }
}
// 非可換のcompressを方向付き文字列で検証（枝は数えるだけ）。
struct Ordered {
    using Info=char;
    struct Point{int count;};
    struct Path{std::string text;int off;};
    static Path vertex(char x){return {std::string(1,x),0};}
    static Path add_vertex(Point p,char x){return {std::string(1,x),p.count};}
    static Point add_edge(const Path& p){return {int(p.text.size())+p.off};}
    static Point rake(Point a,Point b){return {a.count+b.count};}
    static Path compress(const Path& a,const Path& b){return {a.text+b.text,a.off+b.off};}
};
void dp_random(){
    using Mint=cp::StaticModInt<12>;
    std::mt19937 rng(89318);
    for(int trial=0;trial<50;++trial){
        int n=9;Forest f(n);std::vector<long long>a(n,1);std::vector<Mint>w(n,Mint(1));std::vector<char> letters(n);
        for(int i=0;i<n;++i)letters[i]=char('a'+i);
        cp::FullyDynamicIndependentSet<Mint> independent(w);
        cp::FullyDynamicDistanceSum<long long> distance(a);
        cp::DynamicTopTree<Ordered> ordered(letters);
        for(int step=0;step<300;++step){
            int u=rng()%n,v=rng()%n;
            if(step%4==0){bool ok=f.cut(u,v);CHECK(independent.cut(u,v)==ok);CHECK(distance.cut(u,v)==ok);CHECK(ordered.cut(u,v)==ok);}
            else if(step%4==1){bool ok=f.link(u,v);CHECK(independent.link(u,v)==ok);CHECK(distance.link(u,v)==ok);CHECK(ordered.link(u,v)==ok);}
            else if(step%4==2){a[u]=rng()%8;w[u]=Mint(a[u]);independent.set_vertex(u,w[u]);distance.set_vertex(u,a[u]);}
            else{independent.make_root(v);distance.make_root(v);ordered.make_root(v);}
            auto vertices=f.subtree(u,u);Mint expected=0;
            for(int mask=0;mask<(1<<n);++mask){bool ok=true;Mint product=1;
                for(int i=0;i<n;++i)if(mask>>i&1){if(std::find(vertices.begin(),vertices.end(),i)==vertices.end()){ok=false;break;}product*=w[i];for(int j=0;j<i;++j)if((mask>>j&1)&&f.edge[i][j])ok=false;}
                if(ok)expected+=product;
            }
            CHECK(independent.query(u)==expected);
            long long want=0;for(int x:vertices)want+=a[x]*static_cast<long long>(f.path(u,x).size()-1);
            CHECK(distance.query(u)==want);
            auto sub=f.subtree(u,v);want=0;for(int x:sub)want+=a[x]*static_cast<long long>(f.path(v,x).size()-1);
            auto got=distance.subtree_query(u,v);CHECK(bool(got)==!sub.empty());if(got)CHECK(*got==want);
            auto path=f.path(u,v);std::string text;for(int x:path)text+=letters[x];auto cluster=ordered.path_cluster(u,v);
            CHECK(bool(cluster)==!path.empty());if(cluster){CHECK(cluster->text==text);CHECK(cluster->off+int(text.size())==int(vertices.size()));}
        }
    }
}
void large(){
    constexpr int n=100000;
    Tree t(A::values(std::vector<long long>(n,1)));
    for(int v=1;v<n;++v)CHECK(t.link(v-1,v));
    CHECK(t.subtree_apply(0,n/2,{0,3}));
    CHECK(t.component_prod(0).sum==2LL*n);
    CHECK(t.path_apply(0,n-1,{-1,2}));CHECK(t.component_prod(0).sum==0);
    CHECK(t.cut(n/2-1,n/2));CHECK(t.component_prod(n-1).sum==-n/2);
    CHECK(t.link(0,n-1));CHECK(t.component_prod(n/2).sum==0);
    for(int v=1;v<n;++v)if(v!=n/2)CHECK(t.cut(v-1,v));
    CHECK(t.cut(0,n-1));
    for(int v=1;v<n;++v)CHECK(t.link(0,v));
    for(int i=0;i<1000;++i){CHECK(t.subtree_apply(1,0,{0,i}));CHECK(t.subtree_prod(1,0)->sum==1LL*(n-1)*i);}
    CHECK(t.allocated_nodes()<=2*n);
}
int main(){
    Tree empty;CHECK(empty.size()==0);bool threw=false;try{empty.get(0);}catch(const std::out_of_range&){threw=true;}CHECK(threw);
    lazy_random();dp_random();large();
    std::cout<<"dynamic: 150000 mixed operations, 15000 DP steps, path order, ownership, 100000-vertex chain/star passed\n";
}
