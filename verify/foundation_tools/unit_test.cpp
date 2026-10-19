#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "basic_utility/big_rational.hpp"
#include "basic_utility/arena.hpp"
#include "basic_utility/bit_operations.hpp"
#include "basic_utility/grid.hpp"
#include "basic_utility/rollback_array.hpp"
#include "basic_utility/radix_sort.hpp"
#include "graph/grid_bfs.hpp"
#include "linear_algebra/matrix_product.hpp"
#include "range_query/monoid_catalog.hpp"
void check(bool ok){if(!ok)throw std::runtime_error("foundation tools check failed");}
template<class E,class F>void throws(F f){bool caught=false;try{f();}catch(const E&){caught=true;}check(caught);}
using R=cp::BigRational;
void fractions(){
    std::vector<std::pair<int,int>> raw;for(int n=-8;n<=8;n++)for(int d=-8;d<=8;d++)if(d)raw.emplace_back(n,d);
    for(auto [a,b]:raw)for(auto [c,d]:raw){
        R x(a,b),y(c,d);
        auto verify=[](const R& value,long long n,long long den){if(den<0){n=-n;den=-den;}auto g=std::gcd(n,den);check(value.numerator()==n/g&&value.denominator()==den/g);};
        verify(x+y,a*d+c*b,b*d);verify(x-y,a*d-c*b,b*d);verify(x*y,a*c,b*d);if(c)verify(x/y,a*d,b*c);
        auto left=static_cast<long long>(a)*d,right=static_cast<long long>(c)*b;if(b*d<0){left=-left;right=-right;}
        check((x<y)==(left<right)&&(x==y)==(left==right)&&(x>y)==(left>right));
        check((x<=y)==(left<=right)&&(x>=y)==(left>=right)&&(x!=y)==(left!=right));
    }
    R value(2,3);auto copy=value;copy+=copy;check(copy==R(4,3));copy-=copy;check(!copy&&copy.to_string()=="0/1");
    copy=value;copy*=copy;check(copy==R(4,9));copy/=copy;check(copy==1);
    throws<std::domain_error>([]{R(1,0);});throws<std::domain_error>([&]{value/=0;});check(value==R(2,3));
    for(auto s:{"","/","1/","/2","1/2/3"," 1/2","1 /2","0x1/2","1.5"})throws<std::invalid_argument>([&]{R::parse(s);});
    check(R::parse("+0006/-0008")==R(-3,4));check(R::parse("-0/-2").to_string()=="0/1");
    std::istringstream in("-6/-8 5 2/0 7/9");in>>value;check(value==R(3,4));in>>value;check(value==5);in>>value;check(in.fail()&&value==5);in.clear();in>>value;check(value==R(7,9));
    auto moved=std::move(value);check(value==0&&moved==R(7,9));value=moved;auto& alias=value;value=alias;check(value==moved);
    cp::Matrix<R> m{{R(1,2),R(1,3)},{R(0),R(1)}};auto square=cp::matrix_multiply(m,m);check(square(0,0)==R(1,4)&&square(0,1)==R(1,2));
    using F=cp::AffineFunctionMonoid<R>;check(F::op({R(1,2),R(1,3)},{R(2),R(3)})(R(1))==R(14,3));
}
struct alignas(128) Tracked {
    inline static int alive=0;inline static std::vector<int> destroyed;
    int value;explicit Tracked(int x):value(x){if(x<0)throw std::runtime_error("constructor failed");++alive;}
    ~Tracked(){--alive;destroyed.push_back(value);}
};
struct Node{long long sum;std::size_t left,right;};
void arenas(){
    Tracked::destroyed.reserve(10000);
    {cp::Arena<Tracked,3> a;check(a.empty());const auto id=a.emplace(0);auto* p=&a[id];
        for(int i=1;i<2000;i++)a.emplace(i);
        check(p==&a[0]&&p->value==0&&Tracked::alive==2000);
        for(std::size_t i=0;i<a.size();i++)check(reinterpret_cast<std::uintptr_t>(&a[i])%128==0);
        throws<std::runtime_error>([&]{a.emplace(-1);});check(a.size()==2000);
        throws<std::out_of_range>([&]{a[a.size()];});check(std::as_const(a)[1].value==1);
        a.clear();check(a.size()==0&&a.capacity()==0&&Tracked::alive==0);
        for(int i=0;i<2000;i++)check(Tracked::destroyed[i]==1999-i);
        check(a.emplace(7)==0);
    }check(Tracked::alive==0);
    cp::Arena<Node,4> arena;
    const auto build=[&](auto&& self,int l,int r)->std::size_t{if(r-l==1)return arena.emplace(Node{0,0,0});auto a=self(self,l,(l+r)/2),b=self(self,(l+r)/2,r);return arena.emplace(Node{0,a,b});};
    const auto update=[&](auto&& self,std::size_t id,int l,int r,int pos,int x)->std::size_t{auto next=arena[id];if(r-l==1)next.sum=x;else{int m=(l+r)/2;if(pos<m)next.left=self(self,next.left,l,m,pos,x);else next.right=self(self,next.right,m,r,pos,x);next.sum=arena[next.left].sum+arena[next.right].sum;}return arena.emplace(next);};
    const auto query=[&](auto&& self,std::size_t id,int l,int r,int pos)->long long{if(r-l==1)return arena[id].sum;int m=(l+r)/2;return pos<m?self(self,arena[id].left,l,m,pos):self(self,arena[id].right,m,r,pos);};
    std::vector<std::size_t> roots{build(build,0,16)};std::vector<std::vector<int>> expected(1,std::vector<int>(16));std::mt19937 rng(131026);
    for(int i=0;i<400;i++){auto base=rng()%roots.size();int pos=rng()%16,x=rng()%1000;auto a=expected[base];a[pos]=x;roots.push_back(update(update,roots[base],0,16,pos,x));expected.push_back(a);}
    for(std::size_t i=0;i<roots.size();i++){check(arena[roots[i]].sum==std::accumulate(expected[i].begin(),expected[i].end(),0LL));for(int p=0;p<16;p++)check(query(query,roots[i],0,16,p)==expected[i][p]);}
    cp::Arena<long long> large;for(int i=0;i<200000;i++)large.emplace(i);check(large[199999]==199999);
}
void bits(){
    check(cp::topbit(0)==-1&&cp::lowbit(0)==-1&&!cp::next_same_popcount(0));
    for(std::uint64_t x=0;x<(1<<16);x++){
        int top=-1,low=-1,count=0;for(int i=0;i<64;i++)if(x&(UINT64_C(1)<<i)){top=i;if(low<0)low=i;++count;}
        check(cp::topbit(x)==top&&cp::lowbit(x)==low&&cp::popcount(x)==count);
    }
    for(int n=0;n<=16;n++)for(int k=0;k<=n;k++){
        std::vector<std::uint64_t> expected,actual;for(std::uint64_t x=0;x<(UINT64_C(1)<<n);x++){int count=0;for(int i=0;i<n;i++)count+=int((x>>i)&1);if(count==k)expected.push_back(x);}
        for(auto x:cp::FixedPopcountMasks(n,k))actual.push_back(x);
        check(actual==expected);
    }
    for(int i=0;i<64;i++)check(cp::topbit(UINT64_C(1)<<i)==i&&cp::lowbit(UINT64_C(1)<<i)==i);
    std::vector<std::uint64_t> single;for(auto x:cp::FixedPopcountMasks(64,1))single.push_back(x);check(single.size()==64&&single.back()==(UINT64_C(1)<<63));
    int count=0;for(auto x:cp::FixedPopcountMasks(64,63)){check(cp::popcount(x)==63);++count;}check(count==64);
    auto all=cp::FixedPopcountMasks(64,64);check(*all.begin()==UINT64_MAX&&!cp::next_same_popcount(UINT64_MAX));
    throws<std::invalid_argument>([]{cp::FixedPopcountMasks(65,1);});throws<std::invalid_argument>([]{cp::FixedPopcountMasks(2,3);});
}
void grids(){
    for(int h=0;h<=8;h++)for(int w=0;w<=8;w++){
        std::vector<std::vector<int>> a(h,std::vector<int>(w));for(int r=0;r<h;r++)for(int c=0;c<w;c++)a[r][c]=100*r+c;
        const auto t=cp::grid_transpose(a),rot=cp::grid_rotate_clockwise(a);check(t.size()==std::size_t(h?w:0));
        for(int c=0;c<w;c++)for(int r=0;r<h;r++){check(t[c][r]==a[r][c]);check(rot[c][h-1-r]==a[r][c]);}
        if(h&&w){check(cp::grid_rotate_clockwise(a,4)==a);check(cp::grid_rotate_clockwise(rot,-1)==a);}
        cp::GridShape shape(h,w);
        for(int r=0;r<h;r++)for(int c=0;c<w;c++)for(int degree:{4,8}){
            auto neighbors=degree==4?shape.neighbors4(r,c):shape.neighbors8(r,c);std::sort(neighbors.begin(),neighbors.end());std::vector<std::pair<int,int>> want;
            for(int y=0;y<h;y++)for(int x=0;x<w;x++){int dr=std::abs(y-r),dc=std::abs(x-c);if((degree==4?dr+dc==1:std::max(dr,dc)==1))want.emplace_back(y,x);}
            check(neighbors==want);
        }
    }
    for(int mask=0;mask<512;mask++){
        std::vector<std::string> a(3,std::string(3,'.'));int d[9][9];
        for(int i=0;i<9;i++){if(mask>>i&1)a[i/3][i%3]='#';for(int j=0;j<9;j++)d[i][j]=999;}
        for(int i=0;i<9;i++)if(a[i/3][i%3]!='#')for(int j=0;j<9;j++)if(a[j/3][j%3]!='#'){
            if(i==j)d[i][j]=0;else if(std::abs(i/3-j/3)+std::abs(i%3-j%3)==1)d[i][j]=1;
        }
        for(int k=0;k<9;k++)for(int i=0;i<9;i++)for(int j=0;j<9;j++)d[i][j]=std::min(d[i][j],d[i][k]+d[k][j]);
        for(int source=0;source<9;source++){auto actual=cp::grid_bfs(a,source/3,source%3);for(int v=0;v<9;v++)check(actual[v/3][v%3]==(d[source][v]==999?-1:d[source][v]));}
    }
    check(cp::grid_bfs({},99,99).empty());check(cp::grid_bfs({"",""},99,99).size()==2);
    throws<std::invalid_argument>([]{cp::grid_transpose(std::vector<std::string>{"ab","a"});});
    throws<std::invalid_argument>([]{cp::grid_bfs({"ab","a"},0,0);});
    throws<std::out_of_range>([]{cp::grid_bfs({"."},1,0);});throws<std::out_of_range>([]{cp::GridShape(0,0).neighbors4(0,0);});
    throws<std::invalid_argument>([]{cp::GridShape(-1,0);});
}
struct ThrowCopy{
    inline static int copies=1000000;int x;
    explicit ThrowCopy(int value):x(value){}
    ThrowCopy(const ThrowCopy& other):x(other.x){if(copies--==0)throw std::runtime_error("copy failed");}
    friend void swap(ThrowCopy& a,ThrowCopy& b) noexcept {std::swap(a.x,b.x);}
};
void rollback(){
    cp::RollbackArray<std::pair<int,int>> a(12,{0,0});auto root=a.snapshot();
    using Snapshot=decltype(a)::Snapshot;std::vector<Snapshot> snapshots{root};std::vector<std::vector<std::pair<int,int>>> expected{a.values()};std::mt19937 rng(161026);
    auto current=a.values();
    for(int i=0;i<10000;i++){
        if(rng()%4==0){auto k=rng()%snapshots.size();a.rollback(snapshots[k]);current=expected[k];snapshots.resize(k+1);expected.resize(k+1);}
        else{auto p=rng()%12;a.modify(p,[&](auto& value){value.first+=int(rng()%21)-10;value.second=i;current[p]=value;});}
        check(a.values()==current);
        if(rng()%3==0){snapshots.push_back(a.snapshot());expected.push_back(current);}
    }
    a.rollback(root);a.set(0,{1,2});auto stale=a.snapshot();a.rollback(root);a.set(0,{3,4});throws<std::invalid_argument>([&]{a.rollback(stale);});
    cp::RollbackArray<std::pair<int,int>> other(12,{0,0});throws<std::invalid_argument>([&]{other.rollback(root);});
    a.clear_history();throws<std::invalid_argument>([&]{a.rollback(root);});check(a[0]==std::make_pair(3,4));
    throws<std::out_of_range>([&]{a.set(12,{0,0});});
    throws<std::runtime_error>([&]{a.modify(0,[](auto& x){x={9,9};throw std::runtime_error("callback");});});check(a[0]==std::make_pair(3,4));
    cp::RollbackArray<bool> flags(3,false);auto s=flags.snapshot();flags.set(1,true);flags.rollback(s);check(!flags[1]);
    cp::RollbackArray<ThrowCopy> throwing(std::vector<ThrowCopy>{ThrowCopy(1)});ThrowCopy value(2);ThrowCopy::copies=1;
    throws<std::runtime_error>([&]{throwing.set(0,value);});check(throwing[0].x==1&&throwing.history_size()==0);ThrowCopy::copies=1000000;
}
template<class T>void sorts(){
    std::mt19937_64 rng(sizeof(T)+171026);
    for(int trial=0;trial<150;trial++){
        std::vector<std::pair<T,int>> a;int n=rng()%300;
        for(int i=0;i<n;i++){T key=static_cast<T>(rng());if constexpr(sizeof(T)>8)key=static_cast<T>((static_cast<__uint128_t>(rng())<<64)|rng());if(i%3==0)key=T(i%5);a.emplace_back(key,i);}
        a.emplace_back(std::numeric_limits<T>::lowest(),n);a.emplace_back(std::numeric_limits<T>::max(),n+1);
        auto expected=a;std::stable_sort(expected.begin(),expected.end(),[](const auto& x,const auto& y){return x.first<y.first;});
        int calls=0;cp::radix_sort_by(a,[&](const auto& x){++calls;return x.first;});check(a==expected&&calls==int(a.size()));
    }
}
void radix(){
    sorts<signed char>();sorts<unsigned char>();sorts<short>();sorts<unsigned short>();sorts<int>();sorts<unsigned>();sorts<long long>();sorts<unsigned long long>();sorts<__int128_t>();sorts<__uint128_t>();
    std::vector<int> empty;cp::radix_sort(empty);std::vector<int> values{3,1,2};
    throws<std::runtime_error>([&]{cp::radix_sort_by(values,[](int){throw std::runtime_error("key failed");return 0;});});check(values==std::vector<int>({3,1,2}));
    std::vector<ThrowCopy> records{ThrowCopy(3),ThrowCopy(1)};ThrowCopy::copies=0;
    throws<std::runtime_error>([&]{cp::radix_sort_by(records,[](const auto& value){return value.x;});});check(records[0].x==3&&records[1].x==1);ThrowCopy::copies=1000000;
    std::vector<long long> large(200000);std::iota(large.rbegin(),large.rend(),-100000LL);cp::radix_sort(large);check(std::is_sorted(large.begin(),large.end()));
}
int main(){fractions();arenas();bits();grids();rollback();radix();std::cout<<"73984 rational pairs; arena persistence/lifetime; 16-bit masks; 512 grid BFS cases; 10000 rollback operations; 1500 stable radix cases passed\n";}
