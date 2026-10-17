#include <algorithm>
#include <climits>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include "range_query/all.hpp"
#include "static_query/all.hpp"
#include "offline_query/compressed_fenwick_tree_2d.hpp"
void check(bool ok) {if(!ok) throw std::runtime_error("range differential mismatch");}
long long add(long long a,long long b){return a+b;} long long zero(){return 0;}
std::string concat(std::string a,std::string b){return a+b;} std::string blank(){return {};}
using A=cp::AffineSumAction<long long>; using D=cp::DynamicLazySegtree<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id,A::zero>;
int main(){
    std::mt19937 rng(87123);
    for(int n=0;n<=65;++n) {
        std::vector<long long>a(n); std::vector<std::string> words(n);
        for(int i=0;i<n;++i){a[i]=int(rng()%21)-10;words[i]=char('a'+i%26);}
        cp::DisjointSparseTable<long long,add,zero> dst(a); cp::DisjointSparseTable<std::string,concat,blank> ordered(words);
        for(int l=0;l<=n;++l) for(int r=l;r<=n;++r) {
            check(dst.prod(l,r)==std::accumulate(a.begin()+l,a.begin()+r,0LL));
            check(ordered.prod(l,r)==std::accumulate(words.begin()+l,words.begin()+r,std::string{}));
        }
        cp::DynamicSegtree<long long,add,zero> seg(n); D lazy(n); cp::ActionLazySegtree<A> dense(A::values(std::vector<long long>(n,0)));
        std::fill(a.begin(),a.end(),0);
        for(int step=0;step<200;++step){
            int l=rng()%(n+1),r=rng()%(n+1);if(l>r)std::swap(l,r);
            A::F f{int(rng()%3)-1,int(rng()%11)-5};lazy.apply(l,r,f);dense.apply(l,r,f);
            for(int i=l;i<r;++i) {a[i]=f.a*a[i]+f.b;seg.set(i,a[i]);}
            l=rng()%(n+1);r=rng()%(n+1);if(l>r)std::swap(l,r);
            auto expected=std::accumulate(a.begin()+l,a.begin()+r,0LL);
            check(seg.prod(l,r)==expected && lazy.prod(l,r).sum==expected && dense.prod(l,r).sum==expected);
        }
    }
    using Add=cp::AddSumAction<long long>;using Assign=cp::AssignSumAction<long long>;
    cp::ActionLazySegtree<Add> adding(Add::values({1,2,3}));adding.apply(0,3,4);check(adding.all_prod().sum==18);
    cp::ActionLazySegtree<Assign> setting(Assign::values({1,2,3}));setting.apply(0,3,0LL);setting.apply(1,2,7LL);check(setting.all_prod().sum==7);
    cp::DynamicSegtree<std::string,concat,blank> order(LLONG_MAX);order.set(LLONG_MAX-1,"b");order.set(0,"a");check(order.all_prod()=="ab");
    D huge(LLONG_MAX);huge.apply(LLONG_MAX-1,LLONG_MAX,{0,7});check(huge.get(LLONG_MAX-1).sum==7);check(huge.prod(0,1).sum==0);
    check(huge.node_count()<400);
    for(int h=0;h<=9;++h) for(int w=0;w<=9;++w){
        std::vector<std::vector<long long>> a(h,std::vector<long long>(w));cp::FenwickTree2D<long long> bit(h,w);cp::Segtree2D<long long,add,zero> seg(h,w);
        std::vector<std::pair<long long,long long>> points;for(int x=0;x<h;++x)for(int y=0;y<w;++y)points.emplace_back(x*7-20,y*9-30);
        cp::CompressedFenwickTree2D<long long> compressed(points);
        for(int step=0;step<200;++step){
            if(h&&w){int x=rng()%h,y=rng()%w;long long d=int(rng()%11)-5;a[x][y]+=d;bit.add(x,y,d);seg.set(x,y,a[x][y]);compressed.add(x*7-20,y*9-30,d);}
            int l=rng()%(h+1),r=rng()%(h+1),d=rng()%(w+1),u=rng()%(w+1);if(l>r)std::swap(l,r);if(d>u)std::swap(d,u);
            long long sum=0;for(int x=l;x<r;++x)for(int y=d;y<u;++y)sum+=a[x][y];
            check(bit.sum(l,d,r,u)==sum && seg.prod(l,d,r,u)==sum && compressed.sum(l*7-20,d*9-30,r*7-20,u*9-30)==sum);
        }
        cp::Segtree2D<long long,add,zero> built(a);check(built.prod(0,0,h,h?w:0)==bit.sum(h,w));
    }
    cp::CompressedFenwickTree2D<long long> extremal({{LLONG_MIN,LLONG_MIN},{LLONG_MAX,0},{0,0}});extremal.add(LLONG_MIN,LLONG_MIN,3);check(extremal.sum(0,0)==3);
    bool thrown=false;try{extremal.add(0,LLONG_MIN,99);}catch(const std::invalid_argument&){thrown=true;}check(thrown && extremal.sum(1,1)==3);
    std::cout<<"range/static: all DST intervals, noncommutative products, affine composition, huge domains and 100 grids passed\n";
}
