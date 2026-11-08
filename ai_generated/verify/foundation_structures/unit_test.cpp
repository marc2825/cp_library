#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>
#include "basic_utility/hash_map.hpp"
#include "basic_utility/static_modint.hpp"
#include "basic_utility/modint61.hpp"
#include "graph/csr_graph.hpp"
#include "graph_basic/bfs.hpp"
#include "range_query/monoid_catalog.hpp"
#include "range_query/polynomial_add_sum.hpp"

void check(bool ok) { if (!ok) throw std::runtime_error("foundation structures check failed"); }
template <class E, class F> void throws(F f) {
    bool caught = false; try { f(); } catch (const E&) { caught = true; } check(caught);
}
struct ConstantHash { int operator()(int) const { return 7; } };
struct FoldHash { std::size_t operator()(int x) const { return unsigned(x) / 10; } };
struct FoldEqual { bool operator()(int a, int b) const { return a / 10 == b / 10; } };
struct Tracked {
    inline static int alive = 0, copies_left = 1000000;
    int value;
    explicit Tracked(int v) : value(v) { ++alive; }
    Tracked(const Tracked& x) : value(x.value) {
        if (!copies_left--) throw std::runtime_error("copy failure");
        ++alive;
    }
    Tracked& operator=(const Tracked&) = default;
    ~Tracked() { --alive; }
};
template <class Hash> void maps(Hash hash) {
    cp::HashMap<int, int, Hash> actual(hash), replay(hash);
    std::map<int,int> expected;
    std::mt19937 rng(8102026);
    for (int i = 0; i < 30000; ++i) {
        const int key = int(rng()%251)-125, value = int(rng()%10000);
        switch (rng()%8) {
        case 0: {
            const auto a = actual.try_emplace(key,value);
            const auto b = expected.emplace(key,value);
            replay.try_emplace(key,value); check(a.second == b.second && *a.first == b.first->second); break;
        }
        case 1: actual.insert_or_assign(key,value); replay.insert_or_assign(key,value); expected[key]=value; break;
        case 2: check(actual.erase(key) == (expected.erase(key)!=0)); replay.erase(key); break;
        case 3: ++actual[key]; ++replay[key]; ++expected[key]; break;
        case 4: actual.reserve(rng()%400); replay.reserve(actual.capacity()/2); break;
        case 5: {
            auto* p=actual.find(key); check(bool(p)==bool(expected.count(key)));
            if(p) check(*p==expected.at(key));
            break;
        }
        case 6: if(i%997==0){actual.clear();replay.clear();expected.clear();} break;
        default: check(actual.contains(key)==bool(expected.count(key)));
        }
        check(actual.size()==expected.size() && actual.size()<=actual.capacity()/2);
        if(i%127==0){
            std::map<int,int> observed;
            std::as_const(actual).for_each([&](const int& k,const int& v){observed.emplace(k,v);});
            check(observed==expected);
            std::vector<std::pair<int,int>> a,b;
            actual.for_each([&](const auto& k,auto& v){static_assert(std::is_const_v<std::remove_reference_t<decltype(k)>>);a.emplace_back(k,v);});
            replay.for_each([&](const auto& k,auto& v){b.emplace_back(k,v);});
            // reserve may have rebuilt one table earlier; contents still reproduce.
            std::sort(a.begin(),a.end());std::sort(b.begin(),b.end());check(a==b);
        }
    }
    auto copy=actual; actual.clear();check(copy.size()==expected.size());
    auto moved=std::move(copy);check(copy.empty());copy[999]=1;check(copy.at(999)==1);
    actual=moved; moved.clear();check(actual.size()==expected.size());
    auto& same=actual;actual=same;check(actual.size()==expected.size());
    throws<std::out_of_range>([&]{actual.at(10000);});
    throws<std::length_error>([&]{actual.reserve(std::numeric_limits<std::size_t>::max());});
    throws<std::length_error>([&]{actual.rehash(std::numeric_limits<std::size_t>::max());});
    actual.rehash(0);check(actual.size()==expected.size());
    std::as_const(actual).for_each([&](int k,int v){check(expected.at(k)==v);});
}
void hash_maps() {
    maps(cp::SeededHash(0));maps(ConstantHash{});
    cp::SeededHash h(123), same(123), other(456);
    auto key=std::make_tuple(-1,std::pair<int,int>{2,3},std::vector<int>{4,5});
    check(h(key)==same(key));check(h(key)!=other(key));check(h(std::tuple<>{})==same(std::tuple<>{}));
    check(h.hash64(std::string("\xFF\0x",3))==h.hash64(std::vector<unsigned char>{255,0,'x'}));
    check(h.hash64(std::numeric_limits<__int128_t>::min())==same.hash64(std::numeric_limits<__int128_t>::min()));
    check(h.hash64(__uint128_t(1)<<64)!=h.hash64(__uint128_t(0)));
    cp::HashMap<decltype(key),std::string> composite(h);
    composite.try_emplace(key,3,'x');check(composite.at(key)=="xxx");
    cp::HashMap<int,int,FoldHash,FoldEqual> grouped;
    check(grouped.insert(21,1).second);check(!grouped.insert(29,2).second);check(grouped.at(25)==1);
    check(grouped.erase(20)&&grouped.empty());
    cp::HashMap<int,std::string> aliases(cp::SeededHash(7));
    for(int i=0;i<4;i++)aliases[i]="value";
    aliases.insert(10,aliases.at(0));check(aliases.at(10)=="value");
    {
        cp::HashMap<int,Tracked> table(cp::SeededHash(9));
        table.try_emplace(1,8);table.try_emplace(2,9);
        Tracked::copies_left=0;
        check(!table.try_emplace(1,7).second); // Existing key does not construct a value.
        throws<std::runtime_error>([&]{table.rehash(128);});
        check(table.size()==2&&table.at(1).value==8&&Tracked::alive==2);
        Tracked::copies_left=1000000;table.clear();check(Tracked::alive==0);
    }
    check(Tracked::alive==0);
    cp::HashMap<int,int,ConstantHash> collide;
    for(int i=0;i<2000;i++)collide[i]=i;
    for(int i=0;i<2000;i+=2)check(collide.erase(i));
    for(int i=1;i<2000;i+=2)check(collide.at(i)==i);
    for(int i=0;i<2000;i+=2)collide[i]=-i;
    for(int i=0;i<2000;i++)check(collide.at(i)==(i%2?i:-i));
    cp::HashMap<std::uint64_t,int> large(cp::SeededHash(1));
    for(std::uint64_t i=0;i<200000;i++)large[i<<32]=int(i);
    for(std::uint64_t i=0;i<200000;i++)check(large.at(i<<32)==int(i));
}

void graphs() {
    std::mt19937 rng(9026);
    for(int trial=0;trial<500;trial++){
        const int n=int(rng()%35);const auto kind=trial%2?cp::GraphKind::Directed:cp::GraphKind::Undirected;
        cp::Graph<int> original(n,kind);std::vector<cp::Edge<int>> edges;
        if(n)for(int i=0,m=int(rng()%150);i<m;i++){
            const int u=int(rng()%n),v=int(rng()%n),w=int(rng()%201)-100;
            original.add_edge(u,v,w);edges.push_back({u,v,w,999});
        }
        cp::CSRGraph<int> actual(n,edges,kind),converted(original);
        check(actual.offsets()==converted.offsets());check(actual.edge_count()==original.edge_count());
        auto back=actual.to_graph();
        for(int u=0;u<n;u++){
            check(actual[u].size()==original[u].size());
            for(std::size_t j=0;j<actual[u].size();j++){
                const auto& a=actual[u][j];const auto& e=original[u][j];
                check(a.from==e.from&&a.to==e.to&&a.cost==e.cost&&a.idx==e.idx);
                check(back[u][j].to==a.to&&back[u][j].idx==a.idx);
                if(actual.directed())check(a.rev==-1);
                else{
                    const auto& b=actual.arc(a.rev);
                    check(b.to==a.from&&b.from==a.to&&b.cost==a.cost&&b.idx==a.idx);
                    check(&actual.arc(b.rev)==&a&&&a!=&b);
                }
            }
        }
        if(n)check(cp::bfs(original,0).dist==cp::bfs(back,0).dist);
        auto copy=actual;auto moved=std::move(copy);check(copy.size()==0&&copy.edge_count()==0);
        copy=moved;check(copy.offsets()==actual.offsets());
    }
    cp::CSRGraph<int> empty(4);check(empty[0].empty()&&empty[0].begin()==empty[0].end());
    throws<std::out_of_range>([&]{empty[-1];});throws<std::out_of_range>([&]{empty[4];});
    throws<std::out_of_range>([&]{empty[0][0];});throws<std::out_of_range>([&]{empty.arc(0);});
    throws<std::invalid_argument>([]{cp::CSRGraph<> g(-1);});
    throws<std::out_of_range>([]{cp::CSRGraph<> g(1,{{0,1,2}});});
    std::vector<cp::Edge<int>> path;for(int i=1;i<200000;i++)path.push_back({i-1,i,1});
    cp::CSRGraph<int> large(200000,path);check(large.arc_count()==399998&&large[100000].size()==2);
}

template <bool AllowEmpty> void subarrays(const std::vector<long long>& a) {
    using M=cp::MaxSubarrayMonoid<long long,AllowEmpty>;
    std::vector<typename M::S> values;for(auto x:a)values.push_back(M::leaf(x));
    cp::MonoidSegtree<M> tree(values);
    for(int l=0;l<=int(a.size());l++)for(int r=l;r<=int(a.size());r++){
        const auto x=tree.prod(l,r);check(x.length==std::size_t(r-l));
        long long best=(AllowEmpty||l==r)?0:std::numeric_limits<long long>::min();
        for(int i=l;i<r;i++){long long sum=0;for(int j=i;j<r;j++){sum+=a[j];best=std::max(best,sum);}}
        check(x.best==best&&x.sum==std::accumulate(a.begin()+l,a.begin()+r,0LL));
    }
}
using Mint=cp::StaticModInt<101>;
void monoids() {
    for(int n=0;n<=6;n++){
        int limit=1;for(int i=0;i<n;i++)limit*=3;
        for(int mask=0;mask<limit;mask++){
            std::vector<long long>a(n);int x=mask;for(auto&v:a){v=x%3-1;x/=3;}
            subarrays<false>(a);subarrays<true>(a);
            using Min=cp::MinCountMonoid<long long>;using Max=cp::MaxCountMonoid<long long>;
            auto lo=Min::e();auto hi=Max::e();
            for(auto v:a){lo=Min::op(lo,Min::leaf(v));hi=Max::op(hi,Max::leaf(v));}
            if(n){const auto bounds=std::minmax_element(a.begin(),a.end());check(lo.value==*bounds.first&&hi.value==*bounds.second);
                check(lo.count==std::size_t(std::count(a.begin(),a.end(),lo.value)));check(hi.count==std::size_t(std::count(a.begin(),a.end(),hi.value)));}
            else check(lo.count==0&&hi.count==0);
        }
    }
    using F=cp::AffineFunctionMonoid<Mint>;using L=cp::LinearFractionalMonoid<Mint>;using M=cp::MatrixProductMonoid<Mint,2>;
    std::mt19937 rng(1026);
    for(int trial=0;trial<500;trial++){
        std::vector<F::S> fs;std::vector<L::S> ls;std::vector<M::S> matrices;
        for(int i=0;i<8;i++){
            fs.push_back({Mint(rng()),Mint(rng())});
            L::S t;
            do {t={Mint(rng()),Mint(rng()),Mint(rng()),Mint(rng())};}while(t.a*t.d==t.b*t.c);
            ls.push_back(L::make(t.a,t.b,t.c,t.d));matrices.push_back({{t.a,t.b},{t.c,t.d}});
        }
        cp::MonoidSegtree<F> affine(fs);cp::MonoidSegtree<L> fractional(ls);cp::MonoidSegtree<M> matrix(matrices);
        for(int l=0;l<=8;l++)for(int r=l;r<=8;r++){
            Mint x(rng());auto y=x;for(int i=l;i<r;i++)y=fs[i](y);check(affine.prod(l,r)(x)==y);
            std::optional<Mint> q=trial%2?std::optional<Mint>(x):std::nullopt;
            auto want=q;for(int i=l;i<r;i++)want=L::apply(ls[i],want);check(L::apply(fractional.prod(l,r),q)==want);
            std::vector<Mint> vec{x,1},expected=vec;
            for(int i=r;i-->l;){const auto& a=matrices[i];expected={a(0,0)*expected[0]+a(0,1)*expected[1],a(1,0)*expected[0]+a(1,1)*expected[1]};}
            check(cp::matrix_vector_multiply(matrix.prod(l,r),vec)==expected);
        }
        // Point updates must preserve order as well.
        affine.set(3,{2,3});fs[3]={2,3};Mint y=7;for(const auto&f:fs)y=f(y);check(affine.all_prod()(7)==y);
    }
    throws<std::invalid_argument>([]{L::make(1,2,2,4);});
    throws<std::invalid_argument>([]{M::op(cp::Matrix<Mint>(3,3),M::e());});
    using H=cp::RollingHashMonoid<cp::ModInt61,911382323>;
    std::vector<H::S> v;for(unsigned char c:std::string("abacaba"))v.push_back(H::leaf(c+1));
    cp::MonoidSegtree<H> hash(v);
    for(int l=0;l<=7;l++)for(int r=l;r<=7;r++){
        cp::ModInt61 sum=0,power=1;for(int i=l;i<r;i++){sum=sum*911382323+v[i].hash;power*=911382323;}
        const auto got=hash.prod(l,r);check(got.hash==sum&&got.power==power&&got.length==std::size_t(r-l));
    }
}

template <std::size_t D> void polynomial() {
    using A=cp::PolynomialAddSumAction<Mint,D>;
    std::mt19937 rng(121026+D);
    for(int n=0;n<=30;n++){
        std::vector<Mint> a(n);for(auto&v:a)v=Mint(rng());
        cp::ActedMonoidLazySegtree<A> tree(A::values(a));
        for(int trial=0;trial<400;trial++){
            int l=int(rng()%(n+1)),r=int(rng()%(n+1));if(l>r)std::swap(l,r);
            typename A::F f,g;for(auto&v:f)v=Mint(rng());for(auto&v:g)v=Mint(rng());
            auto x=A::leaf(Mint(rng()),Mint(rng())),y=A::leaf(Mint(rng()),Mint(rng()));
            check(A::mapping(f,A::op(x,y)).sum==A::op(A::mapping(f,x),A::mapping(f,y)).sum);
            check(A::mapping(A::composition(f,g),x).sum==A::mapping(f,A::mapping(g,x)).sum);
            check(A::mapping(f,A::e()).sum==Mint(0));
            if(trial%3){tree.apply(l,r,f);for(int i=l;i<r;i++){Mint p=1;for(auto c:f){a[i]+=c*p;p*=Mint(i);}}}
            else{Mint expected=0;for(int i=l;i<r;i++)expected+=a[i];check(tree.prod(l,r).sum==expected);}
            if(n&&trial%17==0){int i=int(rng()%n);a[i]=Mint(rng());tree.set(i,A::leaf(a[i],Mint(i)));}
        }
        for(int i=0;i<n;i++)check(tree.get(i).sum==a[i]);
    }
}
void actions() {
    polynomial<0>();polynomial<1>();polynomial<4>();
    using AP=cp::ArithmeticProgressionAddSumAction<long long>;
    cp::ActedMonoidLazySegtree<AP> ap(AP::values({0,0,0,0,0}));
    ap.apply(1,4,AP::progression(1,3,2));check(ap.all_prod().sum==15&&ap.get(3).sum==7);
    using A=cp::AffineSumAction<Mint>;
    cp::ActedMonoidLazySegtree<A> old(A::values({1,2,3}));
    old.apply(0,3,{2,1});old.apply(1,3,{3,4});check(old.all_prod().sum==47);
}
int main(){hash_maps();graphs();monoids();actions();std::cout<<"hash maps, CSR graphs, monoid folds and polynomial actions passed\n";}
