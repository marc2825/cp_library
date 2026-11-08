#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <vector>
#include "../../union_find/group_potentialized_disjoint_set_union.hpp"
#include "../../union_find/fully_dynamic_connectivity.hpp"
#include "../../range_query/division_segment_tree_beats.hpp"
#include "../../range_query/kinetic_segtree.hpp"
#include "../../range_query/fenwick_tree_nd.hpp"
#include "../../offline_query/orthogonal_range_query.hpp"
#include "../../offline_query/static_range_inversions.hpp"
#include "../../static_query/static_range_mode.hpp"
#include "../../static_query/static_range_lis.hpp"
#define CHECK(x) do { if(!(x)) {std::cerr<<__LINE__<<": "<<#x<<'\n';std::abort();} } while(false)
std::mt19937 rng(354020);
struct Permutations {
    using S=std::array<int,4>;
    static S e(){return {0,1,2,3};}
    static S op(S a,S b){S c;for(int i=0;i<4;++i)c[i]=a[b[i]];return c;}
    static S inverse(S a){S b;for(int i=0;i<4;++i)b[a[i]]=i;return b;}
};
void groups() {
    using G=Permutations;
    for(int trial=0;trial<100;++trial){
        int n=20;std::vector<G::S> potential(n);for(auto& p:potential){p=G::e();std::shuffle(p.begin(),p.end(),rng);}
        cp::GroupPotentializedDisjointSetUnion<G> dsu(n);std::vector<int> label(n);std::iota(label.begin(),label.end(),0);
        for(int step=0;step<100;++step){int a=rng()%n,b=rng()%n;auto diff=G::op(G::inverse(potential[a]),potential[b]);bool same=label[a]==label[b];
            auto result=dsu.merge(a,b,diff);CHECK(result==(same?cp::PotentialMergeResult::AlreadyConsistent:cp::PotentialMergeResult::Merged));
            int old=label[b],now=label[a];for(int& x:label)if(x==old)x=now;
            auto bad=diff;std::swap(bad[0],bad[1]);CHECK(dsu.merge(a,b,bad)==cp::PotentialMergeResult::Contradiction);
            for(int u=0;u<n;++u)for(int v=0;v<n;++v){CHECK(dsu.same(u,v)==(label[u]==label[v]));if(label[u]==label[v])CHECK(*dsu.diff(u,v)==G::op(G::inverse(potential[u]),potential[v]));else CHECK(!dsu.diff(u,v));}
        }
    }
}
void connectivity(){
    for(int trial=0;trial<25;++trial){
        int n=15;cp::FullyDynamicConnectivity dc(n);struct Edge{int a,b;bool active;};std::vector<Edge> edges;
        for(int step=0;step<800;++step){
            if(edges.empty() || rng()%2){int a=rng()%n,b=rng()%n;CHECK(dc.add_edge(a,b)==int(edges.size()));edges.push_back({a,b,true});}
            else {int i=rng()%edges.size();CHECK(dc.remove_edge(i)==edges[i].active);edges[i].active=false;}
            std::vector<int> label(n,-1);int components=0;
            for(int s=0;s<n;++s)if(label[s]<0){std::queue<int> q;q.push(s);label[s]=components;while(!q.empty()){int u=q.front();q.pop();for(auto e:edges)if(e.active){int v=e.a==u?e.b:e.b==u?e.a:-1;if(v>=0 && label[v]<0){label[v]=components;q.push(v);}}}++components;}
            CHECK(dc.component_count()==components);
            for(int a=0;a<n;++a){CHECK(dc.component_size(a)==std::count(label.begin(),label.end(),label[a]));for(int b=0;b<n;++b)CHECK(dc.connected(a,b)==(label[a]==label[b]));}
            if(step%97==0)dc.compact_forest();
        }
    }
}
int lis(const std::vector<int>& a,int l,int r){std::vector<int> tails;for(int i=l;i<r;++i){auto it=std::lower_bound(tails.begin(),tails.end(),a[i]);if(it==tails.end())tails.push_back(a[i]);else *it=a[i];}return int(tails.size());}
void static_queries(){
    for(int n=0;n<=7;++n){int possibilities=1;for(int i=0;i<n;++i)possibilities*=3;
        for(int mask=0;mask<possibilities;++mask){std::vector<int> a(n);int x=mask;for(auto& v:a){v=x%3-1;x/=3;}
            cp::StaticRangeMode<int> mode(a,{},1+mask%9);cp::StaticRangeLIS<int> ranges(a);std::vector<std::pair<int,int>> queries;
            for(int l=0;l<=n;++l)for(int r=l;r<=n;++r){queries.push_back({l,r});CHECK(ranges.query(l,r)==lis(a,l,r));auto m=mode.mode(l,r);if(l==r)CHECK(!m);else {int best=0,value=0;for(int v=-1;v<=1;++v){int f=std::count(a.begin()+l,a.begin()+r,v);if(f>best){best=f;value=v;}}CHECK(m && m->first==value && m->second==best);}}
            auto inv=cp::static_range_inversions(a,queries);for(std::size_t q=0;q<queries.size();++q){auto [l,r]=queries[q];long long expected=0;for(int i=l;i<r;++i)for(int j=i+1;j<r;++j)expected+=a[i]>a[j];CHECK(inv[q]==expected);}
        }
    }
    for(int n=1;n<=120;++n){std::vector<int>a(n);for(int& x:a)x=int(rng()%100)-50;cp::StaticRangeLIS<int> ranges(a);cp::StaticRangeLIS<int,std::greater<int>> decreasing(a);
        auto negative=a;for(int& x:negative)x=-x;for(int i=0;i<200;++i){int l=rng()%(n+1),r=rng()%(n+1);if(l>r)std::swap(l,r);CHECK(ranges.query(l,r)==lis(a,l,r));CHECK(decreasing.query(l,r)==lis(negative,l,r));}}
    #ifdef _GLIBCXX_DEBUG
    constexpr int large_n=2000;
    #else
    constexpr int large_n=20000;
    #endif
    std::vector<int>a(large_n);std::iota(a.begin(),a.end(),0);cp::StaticRangeLIS<int> increasing(a);CHECK(increasing.query(123,large_n-124)==large_n-247);std::reverse(a.begin(),a.end());cp::StaticRangeLIS<int> decreasing(a);CHECK(decreasing.query(0,large_n)==1);
}
template<std::size_t D> void multidimensional(){
    using Point=std::array<long long,D>;std::vector<Point> points(60);for(auto& p:points)for(auto& x:p)x=int(rng()%11)-5;
    cp::FenwickTreeND<long long,D> bit(points);std::vector<long long> weights(points.size());
    for(int step=0;step<500;++step){int i=rng()%points.size(),delta=int(rng()%21)-10;bit.add(points[i],delta);weights[i]+=delta;Point lo,hi;for(std::size_t k=0;k<D;++k){lo[k]=int(rng()%17)-8;hi[k]=int(rng()%17)-8;if(lo[k]>hi[k])std::swap(lo[k],hi[k]);}
        long long expected=0;int count=0;for(std::size_t j=0;j<points.size();++j){bool inside=true;for(std::size_t k=0;k<D;++k)inside&=lo[k]<=points[j][k] && points[j][k]<hi[k];if(inside){expected+=weights[j];++count;}}
        CHECK(bit.sum(lo,hi)==expected);
        if(step%100==0){cp::OrthogonalRangeQuery<long long,D> stat(points,weights);CHECK(stat.sum(lo,hi)==expected);CHECK(stat.count(lo,hi)==count);}
    }
    Point absent;absent.fill(100);bool threw=false;try{bit.add(absent,1);}catch(const std::invalid_argument&){threw=true;}CHECK(threw);
}
void beats(){
    for(int n=0;n<=60;++n){std::vector<long long>a(n);for(auto&x:a)x=rng()%1000;cp::DivisionSegmentTreeBeats tree(a);
        for(int step=0;step<600;++step){int l=rng()%(n+1),r=rng()%(n+1);if(l>r)std::swap(l,r);long long v=rng()%30;switch(rng()%4){case 0:tree.range_assign(l,r,v);for(int i=l;i<r;++i)a[i]=v;break;case 1:tree.range_add(l,r,v);for(int i=l;i<r;++i)a[i]+=v;break;case 2:tree.range_divide(l,r,v+1);for(int i=l;i<r;++i)a[i]/=v+1;break;default:break;}
            CHECK(tree.range_sum(l,r)==std::accumulate(a.begin()+l,a.begin()+r,0LL));CHECK(tree.range_min(l,r)==(l==r?std::numeric_limits<long long>::max():*std::min_element(a.begin()+l,a.begin()+r)));CHECK(tree.range_max(l,r)==(l==r?0:*std::max_element(a.begin()+l,a.begin()+r)));for(int i=0;i<n;++i)CHECK(tree.get(i)==a[i]);
        }
    }
}
void kinetic(){
    using K=cp::KineticSegtree;
    for(int n=0;n<60;++n){std::vector<K::Line>a(n);for(auto&x:a)x={int(rng()%41)-20,int(rng()%101)-50};K tree(a);long long time=0;
        for(int step=0;step<500;++step){if(n && rng()%3==0){int p=rng()%n;a[p]={int(rng()%41)-20,int(rng()%101)-50};tree.set(p,a[p]);}else{time+=rng()%8;tree.advance(time);}
            auto event=tree.next_event();CHECK(!event || *event>time);
            int l=rng()%(n+1),r=rng()%(n+1);if(l>r)std::swap(l,r);auto result=tree.minimum(l,r);if(l==r)CHECK(!result);else {int best=l;for(int i=l+1;i<r;++i)if(K::Wide(a[i].slope)*time+a[i].intercept<K::Wide(a[best].slope)*time+a[best].intercept)best=i;CHECK(result && result->index==best && result->value==K::Wide(a[best].slope)*time+a[best].intercept);}
        }
    }
    auto high=std::numeric_limits<long long>::max();K tree({{high,high},{-high,-high}});tree.advance(high);CHECK(tree.minimum(0,2)->index==1);bool threw=false;try{tree.advance(0);}catch(const std::invalid_argument&){threw=true;}CHECK(threw);
}
int main(){groups();connectivity();static_queries();multidimensional<1>();multidimensional<2>();multidimensional<3>();multidimensional<5>();beats();kinetic();std::cout<<"advanced queries OK\n";}
