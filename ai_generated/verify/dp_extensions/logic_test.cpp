#include "common.hpp"
#include "dp_optimization/frontier_dp.hpp"
#include "dp_optimization/exact_cover.hpp"
#include "dp_optimization/sat_solver.hpp"
using namespace cp;
std::pair<int,int> frontier_oracle(int n,const std::vector<std::pair<int,int>>&edges){
    int paths=0,cycles=0;
    for(int mask=0;mask<(1<<int(edges.size()));++mask){
        std::vector<int>d(n),p(n);std::iota(p.begin(),p.end(),0);auto root=[&](int v){while(v!=p[v])v=p[v];return v;};
        for(int i=0;i<int(edges.size());++i)if(mask>>i&1){auto[a,b]=edges[i];++d[a];++d[b];p[root(a)]=root(b);}
        bool connected=true,path=true,cycle=true;for(int v=0;v<n;++v){connected=connected&&root(v)==root(0);path=path&&d[v]==(v==0||v==n-1?1:2);cycle=cycle&&d[v]==2;}
        paths+=connected&&path;cycles+=connected&&cycle;
    }
    return {n==1?1:paths,cycles};
}
bool satisfies(const std::vector<std::vector<int>>&clauses,const std::vector<bool>&value){for(const auto&c:clauses){bool ok=false;for(int x:c)ok=ok||(x>0?value[x-1]:!value[-x-1]);if(!ok)return false;}return true;}
int main(){
    std::mt19937 rng(7401);int graphs=0;
    for(int n=1;n<=5;++n){std::vector<std::pair<int,int>>all;for(int a=0;a<n;++a)for(int b=a+1;b<n;++b)all.emplace_back(a,b);for(int mask=0;mask<(1<<int(all.size()));++mask){std::vector<std::pair<int,int>>edges;for(int i=0;i<int(all.size());++i)if(mask>>i&1)edges.push_back(all[i]);auto [paths,cycles]=frontier_oracle(n,edges);CHECK(frontier_hamiltonian_path_count(n,edges,0,n-1)==paths);CHECK(frontier_hamiltonian_cycle_count(n,edges)==cycles);++graphs;}}
    for(int trial=0;trial<200;++trial){int n=2+int(rng()%5);std::vector<std::pair<int,int>>edges;for(int i=0;i<11;++i){int a=int(rng()%n),b=int(rng()%n);if(a!=b)edges.emplace_back(a,b);}auto[p,c]=frontier_oracle(n,edges);std::shuffle(edges.begin(),edges.end(),rng);CHECK(frontier_hamiltonian_path_count(n,edges,0,n-1)==p);CHECK(frontier_hamiltonian_cycle_count(n,edges)==c);}
    std::vector<std::pair<int,int>>ladder;for(int i=0;i<500;++i){ladder.emplace_back(2*i,2*i+1);if(i+1<500){ladder.emplace_back(2*i,2*i+2);ladder.emplace_back(2*i+1,2*i+3);}}CHECK(frontier_hamiltonian_cycle_count(1000,ladder)==1);
    FrontierConnectivityState state{{1,1,0,2},{7,7,-1,3},false};state.normalize();CHECK(state.component==std::vector<int>({0,0,-1,1}));
    CHECK(frontier_connected_degree_count(3,{{0,1}},{1,1,0})==1);CHECK(frontier_hamiltonian_cycle_count(0,{})==0);
    throws<std::invalid_argument>([]{frontier_hamiltonian_cycle_count(2,{{0,0}});});
    for(int trial=0;trial<600;++trial){int primary=1+int(rng()%5),secondary=int(rng()%3),n=int(rng()%12);ExactCover cover(primary,secondary);std::vector<int>rows;
        for(int i=0;i<n;++i){int mask=int(rng()%(1U<<(primary+secondary)))|(1<<int(rng()%primary));rows.push_back(mask);std::vector<int>cols;for(int c=0;c<primary+secondary;++c)if(mask>>c&1)cols.push_back(c);CHECK(cover.add_row(cols)==i);}
        std::set<int>expected,actual;for(int mask=0;mask<(1<<n);++mask){int seen=0;bool good=true;for(int i=0;i<n;++i)if(mask>>i&1){if(seen&rows[i])good=false;seen|=rows[i];}if(good&&(seen&((1<<primary)-1))==(1<<primary)-1)expected.insert(mask);}
        CHECK(cover.enumerate([&](const auto&ids){int mask=0;for(int id:ids){CHECK(!(mask>>id&1));mask|=1<<id;}CHECK(actual.insert(mask).second);return true;})==expected.size());CHECK(actual==expected);CHECK(bool(cover.solve())==!expected.empty());CHECK(cover.enumerate([](const auto&){return false;})==std::min<std::size_t>(1,expected.size()));CHECK(cover.enumerate([](const auto&){return true;},0)==0);
    }
    ExactCover queens(16,30);for(int r=0;r<8;++r)for(int c=0;c<8;++c)queens.add_row({r,8+c,16+r+c,31+r-c+7});CHECK(queens.enumerate([](const auto&){return true;})==92);throws<std::runtime_error>([&]{queens.enumerate([](const auto&)->bool{throw std::runtime_error("callback");});});CHECK(queens.solve()->size()==8);
    CHECK(ExactCover(0).solve()->empty());CHECK(!ExactCover(1).solve());throws<std::invalid_argument>([]{ExactCover(1).add_row({0,0});});
    for(int trial=0;trial<2500;++trial){int n=int(rng()%10);SatSolver solver(n);std::vector<std::vector<int>>clauses;
        for(int i=0;i<int(rng()%22);++i){std::vector<int>c;for(int j=0;j<int(rng()%5)&&n;++j)c.push_back((int(rng()%n)+1)*(rng()%2?1:-1));solver.add_clause(c);clauses.push_back(c);}
        for(int repeat=0;repeat<2;++repeat){std::vector<int>assumptions;if(repeat&&n)assumptions.push_back((int(rng()%n)+1)*(rng()%2?1:-1));auto all=clauses;for(int x:assumptions)all.push_back({x});bool expected=false;for(int mask=0;mask<(1<<n);++mask){std::vector<bool>values(n);for(int v=0;v<n;++v)values[v]=mask>>v&1;expected=expected||satisfies(all,values);}auto model=solver.solve(assumptions);CHECK(bool(model)==expected);if(model)CHECK(satisfies(all,*model));}
    }
    SatSolver pigeon(42);for(int p=0;p<7;++p){std::vector<int>c;for(int h=0;h<6;++h)c.push_back(p*6+h+1);pigeon.add_clause(c);}for(int a=0;a<7;++a)for(int b=a+1;b<7;++b)for(int h=0;h<6;++h)pigeon.add_clause({-(a*6+h+1),-(b*6+h+1)});CHECK(!pigeon.solve());
    SatSolver units(20000);for(int v=1;v<=20000;++v)units.add_clause({v});CHECK(units.solve()->size()==20000);
    SatSolver incremental(1);CHECK(incremental.solve());incremental.add_clause({1,1});CHECK(incremental.solve()->at(0));CHECK(!incremental.solve({-1}));incremental.add_clause({-1});CHECK(!incremental.solve());throws<std::out_of_range>([&]{incremental.add_clause({INT_MIN});});throws<std::out_of_range>([&]{incremental.solve({0});});
    std::cout<<"logic: "<<graphs<<" exhaustive graphs, 200 multigraphs, ladder1000, 600 exact-cover subset oracles, 8 queens, 5000 SAT truth tables, pigeonhole and 20000 units passed\n";
}
