#include "common.hpp"
#include "dp_optimization/a_star.hpp"
#include "dp_optimization/ida_star.hpp"
#include "dp_optimization/bidirectional_search.hpp"
using namespace cp;
void check_path(const StateSearchResult<int>& p,const std::vector<std::vector<StateSearchEdge<int>>>& g,int s,int t) {
    CHECK(p.states.front()==s&&p.states.back()==t&&p.states.size()==p.actions.size()+1);long long cost=0;
    for(int i=0;i<int(p.actions.size());++i){bool found=false;for(auto e:g[p.states[i]])if(e.to==p.states[i+1]&&e.action==p.actions[i]){cost+=e.cost;found=true;break;}CHECK(found);}CHECK(cost==p.cost);
}
int main(){
    std::mt19937 rng(7304);
    for(int trial=0;trial<600;++trial){
        int n=1+int(rng()%8),s=int(rng()%n),t=int(rng()%n);std::vector<std::vector<StateSearchEdge<int>>>g(n);
        std::vector<std::vector<int>>d(n,std::vector<int>(n,100000));for(int i=0;i<n;++i)d[i][i]=0;
        for(int id=0;id<15;++id){int a=int(rng()%n),b=int(rng()%n),w=int(rng()%6);g[a].push_back({b,w,id});d[a][b]=std::min(d[a][b],w);}
        for(int k=0;k<n;++k)for(int i=0;i<n;++i)for(int j=0;j<n;++j)d[i][j]=std::min(d[i][j],d[i][k]+d[k][j]);
        std::vector<int>h(n);for(int i=0;i<n;++i)h[i]=d[i][t]<100000?int(rng()%unsigned(d[i][t]+1)):0;
        auto next=[&](int v){return g[v];};auto goal=[&](int v){return v==t;};auto heuristic=[&](int v){return h[v];};
        auto a=a_star(s,next,goal,heuristic),b=ida_star(s,next,goal,heuristic);
        CHECK(bool(a)==(d[s][t]<100000));CHECK(bool(a)==bool(b));if(a){CHECK(a->cost==d[s][t]&&b->cost==d[s][t]);check_path(*a,g,s,t);check_path(*b,g,s,t);}
    }
    {std::vector<std::vector<StateSearchEdge<int>>>g={{{1,5,0},{2,1,1}},{{3,10,2}},{{1,1,3}}, {}};std::vector<int>h{0,0,11,0};auto p=a_star(0,[&](int v){return g[v];},[](int v){return v==3;},[&](int v){return h[v];});CHECK(p&&p->cost==12&&p->expanded==4);check_path(*p,g,0,3);}
    for(int trial=0;trial<600;++trial){int n=1+int(rng()%60),s=int(rng()%n),t=int(rng()%n);std::vector<std::vector<StateSearchEdge<int>>>g(n),rev(n);for(int id=0;id<140;++id){int a=int(rng()%n),b=int(rng()%n);g[a].push_back({b,1,id});rev[b].push_back({a,1,id});}std::vector<int>d(n,-1),q{s};d[s]=0;for(std::size_t j=0;j<q.size();++j)for(auto e:g[q[j]])if(d[e.to]<0){d[e.to]=d[q[j]]+1;q.push_back(e.to);}auto p=bidirectional_bfs(s,t,[&](int v){return g[v];},[&](int v){return rev[v];});CHECK(bool(p)==(d[t]>=0));if(p){CHECK(p->cost==d[t]);check_path(*p,g,s,t);}}
    auto chain=[](int v){std::vector<StateSearchEdge<int>>out;if(v<20000)out.push_back({v+1,1,v});if(v)out.push_back({v-1,1,v+20000});return out;};
    auto deep=ida_star(0,chain,[](int v){return v==20000;},[](int v){return 20000-v;});CHECK(deep&&deep->cost==20000&&deep->actions.size()==20000);
    auto empty=[](int){return std::vector<StateSearchEdge<int>>{};};CHECK(!a_star(0,empty,[](int){return false;},[](int){return 0;}));CHECK(!ida_star(0,empty,[](int){return false;},[](int){return 0;}));CHECK(bidirectional_bfs(3,3,empty,empty)->states==std::vector<int>{3});
    throws<std::invalid_argument>([&]{a_star(0,chain,[](int){return false;},[](int){return -1;});});
    auto negative=[](int){return std::vector<StateSearchEdge<int>>{{1,-1,0}};};throws<std::invalid_argument>([&]{ida_star(0,negative,[](int){return false;},[](int){return 0;});});
    auto over=[](int v){return std::vector<StateSearchEdge<int>>{{v+1,LLONG_MAX,0}};};throws<std::overflow_error>([&]{a_star(0,over,[](int v){return v==2;},[](int){return 0;});});
    std::cout<<"search: 600 A*/IDA* Floyd comparisons, reopening, 600 directed bidirectional BFS, depth 20000 and boundaries passed\n";
}
