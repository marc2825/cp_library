#include <algorithm>
#include <array>
#include <climits>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>
#include "graph_flow/lower_bound_flow.hpp"
#include "graph_flow/project_selection.hpp"
void check(bool ok){if(!ok)throw std::runtime_error("flow/selection mismatch");}
int main(){
    std::mt19937 rng(891723);
    for(int trial=0;trial<700;++trial){
        int n=2+rng()%4,m=rng()%7;std::vector<std::array<int,4>>edges;cp::LowerBoundFlow<long long>f(n);
        for(int i=0;i<m;++i){int u=rng()%n,v=rng()%n,l=rng()%3,r=l+rng()%3;edges.push_back({u,v,l,r});f.add_edge(u,v,l,r);}
        bool circulation=false,exists=false;long long best=LLONG_MIN;std::vector<long long>flows(m);
        auto visit=[&](auto&& self,int i)->void{if(i<m){for(int x=edges[i][2];x<=edges[i][3];++x){flows[i]=x;self(self,i+1);}return;}std::vector<long long>net(n);for(int e=0;e<m;++e){net[edges[e][0]]+=flows[e];net[edges[e][1]]-=flows[e];}bool all=true,internal=true;for(int v=0;v<n;++v){all=all&&net[v]==0;if(v>0&&v<n-1)internal=internal&&net[v]==0;}circulation|=all;if(internal){exists=true;best=std::max(best,net[0]);}};visit(visit,0);
        auto c=f.feasible_circulation();auto a=f.feasible_flow(0,n-1),b=f.maximum_flow(0,n-1);check(bool(c)==circulation&&bool(a)==exists&&bool(b)==exists);
        auto validate=[&](const std::vector<long long>&flow,bool cycle,long long value){check(flow.size()==edges.size());std::vector<long long>net(n);for(int e=0;e<m;++e){check(edges[e][2]<=flow[e]&&flow[e]<=edges[e][3]);net[edges[e][0]]+=flow[e];net[edges[e][1]]-=flow[e];}for(int v=0;v<n;++v)if(cycle||(v>0&&v<n-1))check(net[v]==0);if(!cycle)check(net[0]==value&&net[n-1]==-value);};
        if(c) { validate(*c,true,0); }
        if(a) { validate(a->flow,false,a->value); }
        if(b){check(b->value==best);validate(b->flow,false,b->value);check(f.maximum_flow(0,n-1)->value==best);}
    }
    for(int trial=0;trial<700;++trial){
        int n=1+rng()%7;cp::ProjectSelection<long long>p(n);std::vector<std::array<long long,2>>unary(n);for(int i=0;i<n;++i){unary[i]={int(rng()%11)-5,int(rng()%11)-5};p.add_cost(i,unary[i][0],unary[i][1]);}
        std::vector<std::array<long long,6>>pair;for(int i=0;i<10;++i){int u=rng()%n,v=rng()%n;long long base=int(rng()%7)-3,a=int(rng()%9)-4,b=int(rng()%9)-4,c=rng()%8;pair.push_back({u,v,base,base+b,base+a+c,base+a+b});p.add_pairwise(u,v,base,base+b,base+a+c,base+a+b);}
        std::vector<std::array<int,2>>imp,force;for(int i=0;i<3;++i){int u=rng()%n,v=rng()%n;p.imply(u,v);imp.push_back({u,v});}if(rng()%2){int v=rng()%n,b=rng()%2;p.force(v,b);force.push_back({v,b});}
        if(rng()%5==0){p.force(0,true);p.force(0,false);force.push_back({0,1});force.push_back({0,0});}
        auto energy=[&](int mask){long long cost=0;for(int v=0;v<n;++v)cost+=unary[v][mask>>v&1];for(auto e:pair)cost+=e[2+2*(mask>>e[0]&1)+(mask>>e[1]&1)];return cost;};
        std::optional<long long>best;for(int mask=0;mask<(1<<n);++mask){bool ok=true;for(auto e:imp)if((mask>>e[0]&1)&&!(mask>>e[1]&1))ok=false;for(auto e:force)if((mask>>e[0]&1)!=e[1])ok=false;if(ok&&(!best||energy(mask)<*best))best=energy(mask);}
        auto answer=p.solve();check(bool(answer)==bool(best));if(answer){check(answer->cost==*best);int mask=0;for(int v=0;v<n;++v)if(answer->selected[v])mask|=1<<v;check(energy(mask)==*best);for(auto e:imp)check(!(mask>>e[0]&1)||(mask>>e[1]&1));for(auto e:force)check((mask>>e[0]&1)==e[1]);}
    }
    cp::LowerBoundFlow<long long>negative(2);negative.add_edge(1,0,1,1);check(negative.maximum_flow(0,1)->value==-1);
    cp::LowerBoundFlow<long long>limit(2);limit.add_edge(0,1,0,LLONG_MAX);check(limit.maximum_flow(0,1)->value==LLONG_MAX);limit.add_edge(0,1,0,1);bool thrown=false;try{limit.maximum_flow(0,1);}catch(const std::overflow_error&){thrown=true;}check(thrown);
    cp::LowerBoundFlow<long long>loop(1);loop.add_edge(0,0,LLONG_MAX,LLONG_MAX);check(loop.feasible_circulation()->front()==LLONG_MAX);
    cp::ProjectSelection<long long>bad(2);thrown=false;try{bad.add_pairwise(0,1,1,0,0,1);}catch(const std::invalid_argument&){thrown=true;}check(thrown&&bad.solve()->cost==0);bad.force(0,true);bad.force(0,false);check(!bad.solve());
    cp::ProjectSelection<long long>minimum(1);minimum.add_cost(0,LLONG_MIN,LLONG_MIN);check(minimum.solve()->cost==LLONG_MIN);
    cp::ProjectSelection<long long>empty(0);check(empty.solve()->cost==0);
    cp::ProjectSelection<long long>large(200000);large.add_profit(0,7);for(int v=1;v<large.size();++v)large.imply(v-1,v);auto solved=large.solve();check(solved->cost==-7&&solved->selected.back());
    std::cout<<"flow: 700 full flow enumerations, 700 binary assignment enumerations, negative values, extrema and 200000 projects passed\n";
}
