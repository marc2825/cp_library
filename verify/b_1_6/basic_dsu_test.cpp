#include "common.hpp"
#include "basic_utility/all.hpp"
#include "union_find/all.hpp"
#include <limits>
#include <queue>

static std::vector<int> components(int n,const std::vector<std::pair<int,int>>& edges,int skipped=-1) {
    std::vector<std::vector<int>> adj(n);
    for(int i=0;i<int(edges.size());++i) if(i!=skipped) { auto [u,v]=edges[i]; adj[u].push_back(v); adj[v].push_back(u); }
    std::vector<int> ids(n,-1);
    for(int s=0;s<n;++s) if(ids[s]<0) {
        std::queue<int> q; q.push(s); ids[s]=s;
        while(!q.empty()) { int u=q.front(); q.pop(); for(int v:adj[u]) if(ids[v]<0) { ids[v]=s; q.push(v); } }
    }
    return ids;
}
int main() {
    for(std::uint64_t n=0;n<5000;++n) for(unsigned k=1;k<=12;++k) {
        auto x=cp::integer_kth_root(n,k);
        auto fits=[&](std::uint64_t a) { std::uint64_t p=1; for(unsigned i=0;i<k;++i) { if(a && p>n/a) return false; p*=a; } return p<=n; };
        CHECK(fits(x) && !fits(x+1));
    }
    auto max=std::numeric_limits<std::uint64_t>::max();
    CHECK(cp::integer_kth_root(max,1)==max); CHECK(cp::integer_kth_root(max,2)==4294967295ULL); CHECK(cp::integer_kth_root(max,64)==1);
    bool rejected=false; try { cp::integer_kth_root(2,0); } catch(const std::invalid_argument&) { rejected=true; } CHECK(rejected);
    for(std::uint64_t p=1;p<=100;++p) for(std::uint64_t q=1;q<=100;++q) {
        auto f=cp::stern_brocot_fraction(cp::stern_brocot_path(p,q)); auto g=std::gcd(p,q); CHECK(f.numerator==p/g && f.denominator==q/g);
    }
    CHECK(cp::stern_brocot_fraction({{'R',max-1}}).numerator==max);
    for(unsigned limit=1;limit<=25;++limit) for(unsigned p=0;p<=25;++p) for(unsigned q=1;q<=12;++q) {
        auto [l,r]=cp::fraction_binary_search(limit,[=](auto a,auto b){return a*q<=p*b;});
        CHECK(l.numerator*q<=p*l.denominator); CHECK(r.numerator*q>p*r.denominator);
        for(unsigned a=0;a<=limit;++a) for(unsigned b=1;b<=limit;++b) {
            if(a*q<=p*b) CHECK(a*l.denominator<=l.numerator*b);
            else CHECK(r.numerator*b<=a*r.denominator);
        }
    }
    auto inf=cp::fraction_binary_search(max,[](auto,auto){return true;}); CHECK(inf.first.numerator==max && inf.first.denominator==1);
    for(int trial=0;trial<600;++trial) {
        std::vector<std::vector<int>> lists(pick(30));
        for(auto& a:lists) { a.resize(pick(60)); for(auto& x:a) x=pick(8); std::sort(a.begin(),a.end()); }
        cp::FractionalCascading<int> fc(lists);
        for(int x=-1;x<=9;++x) { auto result=fc.lower_bounds(x); for(int i=0;i<int(lists.size());++i) CHECK(result[i]==std::lower_bound(lists[i].begin(),lists[i].end(),x)-lists[i].begin()); }
        std::vector<std::string> a(pick(70)); for(auto& s:a) s=char('a'+pick(4));
        cp::SqrtDecomposition<std::string,cat,empty_string> sq(a);
        for(int t=0;t<80;++t) {
            if(!a.empty() && pick(2)) { int p=pick(int(a.size())); a[p]=char('a'+pick(4)); sq.set(p,a[p]); }
            int l=pick(int(a.size())+1),r=pick(int(a.size())+1); if(l>r) std::swap(l,r); std::string want; for(int i=l;i<r;++i) want+=a[i]; CHECK(sq.prod(l,r)==want);
        }
        std::vector<long long> costs(1+pick(100)); for(auto& x:costs) x=pick(100)-50;
        auto best=cp::nonmonotone_argmin<long long>(0,costs.size(),[&](auto i){return costs[i];},[&](auto l,auto r){ return *std::min_element(costs.begin()+l,costs.begin()+r); });
        CHECK(best->first==std::uint64_t(std::min_element(costs.begin(),costs.end())-costs.begin()));
    }
    CHECK(!cp::nonmonotone_argmin<int>(0,0,[](auto){return 0;},[](auto,auto){return 0;}));
    for(int trial=0;trial<180;++trial) {
        int n=1+pick(28); cp::ParallelDisjointSetUnion parallel(n); cp::AdditiveDisjointSetUnion<long long> additive(n);
        cp::DynamicDisjointSetUnion<std::string> dynamic; std::vector<int> groups(n); std::iota(groups.begin(),groups.end(),0); std::vector<long long> values(n);
        for(int i=0;i<n;++i) dynamic.add(std::to_string(i));
        auto merge=[&](int a,int b) { int from=groups[b],to=groups[a]; for(int& g:groups) if(g==from) g=to; additive.merge(a,b); dynamic.merge(std::to_string(a),std::to_string(b)); };
        for(int t=0;t<300;++t) {
            int u=pick(n),v=pick(n),len=pick(std::min(n-u,n-v)+1),type=pick(4); long long delta=pick(19)-9;
            if(type==0) { parallel.merge_ranges(u,v,len); for(int i=0;i<len;++i) merge(u+i,v+i); }
            else if(type==1) { additive.add_component(u,delta); for(int i=0;i<n;++i) if(groups[i]==groups[u]) values[i]+=delta; }
            else if(type==2) { additive.add_vertex(u,delta); values[u]+=delta; }
            else { parallel.merge(u,v); merge(u,v); }
            for(int i=0;i<n;++i) {
                CHECK(additive.get(i)==values[i]); long long total=0; int count=0;
                for(int j=0;j<n;++j) { CHECK(parallel.same(i,j)==(groups[i]==groups[j])); CHECK(additive.same(i,j)==(groups[i]==groups[j])); CHECK(dynamic.same(std::to_string(i),std::to_string(j))==(groups[i]==groups[j])); if(groups[i]==groups[j]) {total+=values[j]; ++count;} }
                CHECK(additive.component_sum(i)==total && additive.size(i)==count && parallel.size(i)==count && dynamic.size(std::to_string(i))==count);
            }
        }
    }
    for(int trial=0;trial<160;++trial) {
        int n=1+pick(12); cp::IncrementalBridgeConnectivity bridge(n); std::vector<std::pair<int,int>> edges;
        for(int t=0;t<60;++t) {
            edges.emplace_back(pick(n),pick(n)); bridge.add_edge(edges.back().first,edges.back().second);
            auto ids=components(n,edges); std::vector<bool> is_bridge(edges.size()); int count=0;
            for(int i=0;i<int(edges.size());++i) { auto other=components(n,edges,i); auto [u,v]=edges[i]; is_bridge[i]=other[u]!=other[v]; count+=is_bridge[i]; }
            CHECK(bridge.bridge_count()==count);
            std::vector<std::pair<int,int>> safe; for(int i=0;i<int(edges.size());++i) if(!is_bridge[i]) safe.push_back(edges[i]); auto blocks=components(n,safe);
            for(int i=0;i<n;++i) for(int j=0;j<n;++j) { CHECK(bridge.same(i,j)==(ids[i]==ids[j])); CHECK(bridge.two_edge_connected(i,j)==(blocks[i]==blocks[j])); }
        }
    }
    cp::RangeSet<long long> ranges;
    std::set<int> covered;
    for(int i=0;i<3000;++i) { int l=pick(80)-40,r=pick(80)-40; if(l>r) std::swap(l,r); bool insert=pick(2); if(insert) ranges.insert(l,r); else ranges.erase(l,r); for(int x=l;x<r;++x) { if(insert) covered.insert(x); else covered.erase(x); } for(int x=-42;x<42;++x) CHECK(ranges.contains(x)==bool(covered.count(x))); }
    std::cout<<"basic/DSU: exhaustive roots/fractions, 600 catalogs, 54000 DSU operations, 9600 graph prefixes passed\n";
}
