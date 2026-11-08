#include "common.hpp"
#include "offline_query/all.hpp"
#include "range_query/range_tree.hpp"
#include <array>
#include <limits>

int main() {
    for(int trial=0;trial<120;++trial) {
        int n=pick(80); std::vector<std::pair<long long,long long>> points;
        for(int i=0;i<n;++i) points.emplace_back(pick(21)-10,pick(21)-10);
        cp::KdTree<long long> kd(points); cp::RangeTree<> tree(points); std::vector<long long> weights(n);
        for(int t=0;t<250;++t) {
            if(n) { int p=pick(n); long long x=pick(21)-10; if(pick(2)) { tree.add(p,x); weights[p]+=x; } else { tree.set(p,x); weights[p]=x; } }
            long long l=pick(25)-12,d=pick(25)-12,r=pick(25)-12,u=pick(25)-12; if(l>r) std::swap(l,r); if(d>u) std::swap(d,u);
            long long sum=0; std::vector<int> ids;
            for(int i=0;i<n;++i) if(l<=points[i].first && points[i].first<r && d<=points[i].second && points[i].second<u) { sum+=weights[i]; ids.push_back(i); }
            CHECK(tree.sum(l,d,r,u)==sum); auto got=kd.report(l,d,r,u); std::sort(got.begin(),got.end()); CHECK(got==ids);
            auto near=kd.nearest(l,d);
            if(!n) CHECK(!near);
            else { int best=-1; long long distance=100000; for(int i=0;i<n;++i) { auto dx=points[i].first-l,dy=points[i].second-d,dist=dx*dx+dy*dy; if(dist<distance) {distance=dist; best=i;} } CHECK(near->first==best && near->second==distance); }
        }
    }
    for(int trial=0;trial<80;++trial) {
        std::vector<std::array<long long,4>> rects;
        for(int i=0;i<50;++i) { long long l=pick(15)-7,r=pick(15)-7,d=pick(15)-7,u=pick(15)-7; if(l>r) std::swap(l,r); if(d>u) std::swap(d,u); rects.push_back({l,d,r,u}); }
        cp::RectangleAddRectangleSum<long long> sum(rects); cp::RectangleAddPointGet<long long> point(rects); long long cells[16][16]={};
        for(int t=0;t<300;++t) {
            auto a=rects[pick(50)]; long long delta=pick(21)-10; sum.add(a[0],a[1],a[2],a[3],delta); point.add(a[0],a[1],a[2],a[3],delta);
            for(int i=int(a[0]);i<a[2];++i) for(int j=int(a[1]);j<a[3];++j) cells[i+8][j+8]+=delta;
            int l=pick(16)-8,r=pick(16)-8,d=pick(16)-8,u=pick(16)-8; if(l>r) std::swap(l,r); if(d>u) std::swap(d,u); long long want=0;
            for(int i=l;i<r;++i) for(int j=d;j<u;++j) want+=cells[i+8][j+8];
            CHECK(sum.sum(l,d,r,u)==want); CHECK(point.get(l,d)==cells[l+8][d+8]);
        }
    }
    for(int trial=0;trial<80;++trial) {
        cp::DynamicRectangleAffineSum<> tree; std::vector<std::array<long long,3>> points;
        for(int t=0;t<650;++t) {
            int type=pick(5); long long l=pick(21)-10,r=pick(21)-10,d=pick(21)-10,u=pick(21)-10; if(l>r) std::swap(l,r); if(d>u) std::swap(d,u);
            if(type==0 || points.empty()) { long long x=pick(21)-10,y=pick(21)-10,w=pick(21)-10; CHECK(tree.add_point(x,y,w)==int(points.size())); points.push_back({x,y,w}); }
            else if(type==1) { long long a=pick(3)-1,b=pick(9)-4; tree.apply(l,d,r,u,a,b); for(auto& p:points) if(l<=p[0] && p[0]<r && d<=p[1] && p[1]<u) p[2]=a*p[2]+b; }
            else if(type==2) { int id=pick(int(points.size())); long long w=pick(21)-10; tree.set(id,w); points[id][2]=w; }
            long long sum=0; for(auto p:points) if(l<=p[0] && p[0]<r && d<=p[1] && p[1]<u) sum+=p[2]; CHECK(tree.sum(l,d,r,u)==sum);
            for(int id=0;id<int(points.size());++id) CHECK(tree.get(id)==points[id][2]);
        }
    }
    for(int bits=0;bits<=7;++bits) {
        int n=1<<bits; std::vector<std::pair<std::uint64_t,std::pair<int,int>>> order;
        for(int x=0;x<n;++x) for(int y=0;y<n;++y) order.push_back({cp::hilbert_order(x,y,bits),{x,y}});
        std::sort(order.begin(),order.end());
        for(int i=0;i<int(order.size());++i) { CHECK(order[i].first==unsigned(i)); if(i) CHECK(std::abs(order[i].second.first-order[i-1].second.first)+std::abs(order[i].second.second-order[i-1].second.second)==1); }
    }
    for(int trial=0;trial<200;++trial) {
        int n=pick(100); std::vector<int> a(n); for(int& x:a) x=pick(20);
        cp::HilbertOrderMo mo(n); std::vector<std::pair<int,int>> queries;
        for(int i=0;i<200;++i) { int l=pick(n+1),r=pick(n+1); if(l>r) std::swap(l,r); mo.add_query(l,r); queries.emplace_back(l,r); }
        auto count=cp::static_range_count_distinct(a,queries); std::vector<int> answer(queries.size()); int freq[20]={},distinct=0;
        mo.run([&](int i){if(freq[a[i]]++==0) ++distinct;},[&](int i){if(--freq[a[i]]==0) --distinct;},[&](int id){answer[id]=distinct;});
        for(int i=0;i<int(queries.size());++i) { auto [l,r]=queries[i]; std::set<int> unique(a.begin()+l,a.begin()+r); CHECK(count[i]==int(unique.size()) && answer[i]==count[i]); }
    }
    cp::OfflineDynamicConnectivity connectivity(3); connectivity.add_edge(0,1); connectivity.add_query(0,2); connectivity.add_edge(1,2); connectivity.add_query(0,2); connectivity.remove_edge(0,1); connectivity.add_query(0,2); CHECK(connectivity.solve()==std::vector<bool>({false,true,false}));
    auto min=std::numeric_limits<long long>::min(),max=std::numeric_limits<long long>::max();
    cp::RectangleAddPointGet<long long> point({{min,min,max,max}}); point.add(min,min,max,max,3); CHECK(point.get(min,min)==3 && point.get(max,max)==0);
    cp::KdTree<long long> kd({{min,min},{max,max}}); CHECK(kd.nearest(min,min)->first==0);
    cp::DynamicRectangleAffineSum<> large;
    for(int i=0;i<10000;++i) { large.add_point(i,i,1); if(i%101==0) large.apply(-1,-1,20000,20000,0,2); }
    CHECK(large.size()==10000);
    std::cout<<"spatial: 30000 rectangle/nearest queries, 24000 rectangle updates, 52000 dynamic affine operations, Hilbert adjacency and 40000 distinct queries passed\n";
}
