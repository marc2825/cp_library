#include <algorithm>
#include <array>
#include <climits>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include "string_algorithm/all.hpp"
#include "game_theory/all.hpp"
#include "heuristic_search/all.hpp"

void check(bool x){if(!x)throw std::runtime_error("final A check failed");}
template <class E,class F> void throws(F f){bool caught=false;try{f();}catch(const E&){caught=true;}check(caught);}
struct ConstantHash { std::size_t operator()(int) const { return 0; } };
int main(){
    std::mt19937_64 random(321325);
    for(int trial=0;trial<400;++trial){
        std::string s;cp::SuffixAutomaton a;
        for(int n=0;n<30;++n){
            std::set<std::string> substrings;
            for(std::size_t l=0;l<s.size();++l)for(std::size_t r=l+1;r<=s.size();++r)substrings.insert(s.substr(l,r-l));
            check(a.distinct_substrings()==substrings.size());check(a.count_occurrences("")==s.size()+1);
            for(const auto& p:substrings){std::size_t count=0;for(std::size_t i=0;i+p.size()<=s.size();++i)count+=s.compare(i,p.size(),p)==0;check(a.contains(p)&&a.count_occurrences(p)==count&&a.first_occurrence(p)==static_cast<int>(s.find(p)));}
            const auto c=static_cast<unsigned char>(random()%5);s.push_back(static_cast<char>(c));a.append(c);
        }
        a.clear();check(a.size()==0&&a.state_count()==1&&a.count_occurrences("")==1);a.append("abc");check(a.distinct_substrings()==6&&!a.contains("d"));
    }
    std::string all_bytes;for(int c=0;c<256;++c)all_bytes.push_back(static_cast<char>(c));cp::SuffixAutomaton bytes(all_bytes);check(bytes.distinct_substrings()==32896);
    auto match=cp::longest_common_substring(all_bytes,all_bytes.substr(120,100));check(match.first_left==120&&match.second_left==0&&match.length==100);
    cp::SuffixAutomaton many(std::string(100000,'x'));check(many.state_count()==100001&&many.count_occurrences("xxx")==99998);
    for(int n=0;n<64;++n){std::string s(n,'a');auto factors=cp::lyndon_factorization(s);check(factors.size()==static_cast<std::size_t>(n));}
    cp::RollingHash2D h({"abcd","efgh","ijkl"}),sub({"fg","jk"});check(h.equal(sub,1,3,1,3,0,2,0,2));check(h.get(1,1,0,4)==cp::RollingHash2D::Hash({0,0}));check(cp::RollingHash2D(std::vector<std::string>(3)).height()==3);
    std::array<std::array<unsigned,64>,64> product{};
    for(unsigned a=0;a<64;++a)for(unsigned b=0;b<64;++b){std::array<bool,256> seen{};for(unsigned x=0;x<a;++x)for(unsigned y=0;y<b;++y)seen[product[x][b]^product[a][y]^product[x][y]]=true;unsigned p=0;while(seen[p])++p;product[a][b]=p;check(cp::nim_product(a,b)==p);}
    for(int t=0;t<600;++t){const auto a=random(),b=random(),c=random();check(cp::nim_product(a,b)==cp::nim_product(b,a));check(cp::nim_product(a,b^c)==(cp::nim_product(a,b)^cp::nim_product(a,c)));check(cp::nim_product(cp::nim_product(a,b),c)==cp::nim_product(a,cp::nim_product(b,c)));check(cp::nim_product(a,0)==0&&cp::nim_product(a,1)==a);}
    for(unsigned k=0;k<6;++k){const auto x=std::uint64_t(1)<<(1u<<k);check(cp::nim_product(x,x)==(x^(x>>1)));}
    auto g=cp::subtraction_grundy({1,3,4,4},1000);check(g.period&&g.period->period==7&&g.value(UINT64_MAX)==g.values[UINT64_MAX%7]);
    check(!cp::subtraction_grundy({1,1000},20).period);check(cp::subtraction_grundy({},1).value(UINT64_MAX)==0);
    for(unsigned mask=0;mask<256;++mask){std::vector<int> moves;for(int i=0;i<8;++i)if(mask>>i&1)moves.push_back(i+1);auto r=cp::subtraction_grundy(moves,80);std::vector<int>w(1000);for(std::size_t i=0;i<w.size();++i){std::set<int>s;for(int x:moves)if(static_cast<std::size_t>(x)<=i)s.insert(w[i-x]);while(s.count(w[i]))++w[i];check((i<80||r.period)?r.value(i)==w[i]:true);}}
    cp::ZobristHash z(100,7,42);std::vector<int> board(100);auto hash=z.hash(board);for(int t=0;t<2000;++t){auto p=random()%board.size();int v=random()%7;hash=z.change(hash,p,board[p],v);board[p]=v;check(hash==z.hash(board));}
    check(cp::ZobristHash(0,1).hash(std::vector<int>{})==0);
    cp::BeamSearchOptions beam;beam.width=3;beam.max_depth=1;
    auto unique=cp::beam_search_unique(0,[](int x){return -x;},[](int){return std::vector<int>{2,1,3,4};},[](int x){return x%3;},beam,ConstantHash{});
    check(unique.frontier.size()==3&&unique.frontier[0].state==4&&unique.frontier[1].state==3&&unique.frontier[2].state==2);
    auto retained=cp::beam_search(0,[](int x){return x;},[](int){return std::vector<int>{1,1,2};},beam);check(retained.frontier[0].state==retained.frontier[1].state);
    cp::TabuSearchOptions tabu;tabu.max_iterations=3;tabu.tenure=1;
    auto t=cp::tabu_search(0,[](int x){return x;},[](int x){return std::vector<int>{1-x};},[](int x){return x;},tabu,ConstantHash{});check(t.current_state==1&&t.best_state==0&&t.accepted==3);
    tabu.tenure=SIZE_MAX;auto exhausted=cp::tabu_search(0,[](int x){return x;},[](int x){return std::vector<int>{1-x};},[](int x){return x;},tabu);check(exhausted.termination==cp::SearchTermination::exhausted&&exhausted.accepted==1);
    auto aspiration=cp::tabu_search(4,[](int x){return x;},[](int x){return std::vector<int>{x-1};},[](int){return 0;},tabu);check(aspiration.best_state==1);
    for(int trial=0;trial<200;++trial){
        cp::Random a(trial),b(trial);cp::AnnealingOptions options;options.max_iterations=100;options.goal=trial%2?cp::SearchGoal::minimize:cp::SearchGoal::maximize;options.start_temperature=5;options.end_temperature=0.1L;
        const auto eval=[](const int& x)->long long{return 1LL*x*x;};
        auto full=cp::simulated_annealing(3,eval,[](const int& x,cp::Random& r){return x+(r.uniform(2)?1:-1);},a,options);
        int evaluations=0,applies=0,undos=0;
        auto delta=cp::simulated_annealing_delta(3,[&](const int& x){++evaluations;return eval(x);},[](const int&,cp::Random& r){return r.uniform(2)?1:-1;},[](const int& x,const int& d)->long long{return 2LL*x*d+d*d;},[&](int& x,int d){x+=d;++applies;},[&](int& x,int d){x-=d;++undos;},b,options);
        check(delta.best_state==full.best_state&&delta.current_state==full.current_state&&delta.best_score==full.best_score&&delta.current_score==full.current_score&&delta.accepted==full.accepted&&delta.iterations==100&&delta.evaluations==101&&evaluations==1&&applies==100&&undos==100-static_cast<int>(delta.accepted)&&a()==b());
    }
    long long state=4,score=16;
    const auto difference=[](const long long& x,const long long& d)->long long{return 2*x*d+d*d;};
    const auto apply=[](long long& x,long long d){x+=d;};const auto undo=[](long long& x,long long d){x-=d;};
    check(!cp::try_scored_move(state,score,1LL,difference,apply,undo,[](const long long&,long long,long long){return false;})&&state==4&&score==16);
    throws<std::runtime_error>([&]{cp::try_scored_move(state,score,1LL,difference,apply,undo,[](const long long&,long long,long long)->bool{throw std::runtime_error("accept failure");});});check(state==4&&score==16);
    score=LLONG_MAX;throws<std::overflow_error>([&]{cp::try_scored_move(state,score,1LL,[](const long long&,const long long&)->long long{return 1;},apply,undo,[](const long long&,long long,long long){return true;});});check(state==4&&score==LLONG_MAX);
    throws<std::invalid_argument>([]{cp::RollingHash2D bad({"ab","x"});});throws<std::invalid_argument>([]{cp::RollingHash2D bad({},256);});throws<std::out_of_range>([&]{h.get(-1,2,0,1);});
    throws<std::invalid_argument>([&]{h.equal(cp::RollingHash2D({"a"},257),0,1,0,1,0,1,0,1);});
    throws<std::invalid_argument>([]{cp::detect_grundy_period({0,1},1);});throws<std::invalid_argument>([]{cp::detect_grundy_period({-1,1});});throws<std::invalid_argument>([]{cp::subtraction_grundy({0},10);});throws<std::invalid_argument>([]{cp::subtraction_grundy({},0);});throws<std::out_of_range>([]{cp::subtraction_grundy({100},5).value(10);});
    throws<std::invalid_argument>([]{cp::ZobristHash bad(1,0);});throws<std::out_of_range>([&]{z.hash(std::vector<int>(100,-1));});throws<std::invalid_argument>([&]{z.hash(std::vector<int>(99));});
    throws<std::invalid_argument>([]{cp::BeamSearchOptions o;o.width=0;cp::beam_search_unique(0,[](int x){return x;},[](int){return std::vector<int>{};},[](int x){return x;},o);});
    cp::SearchOptions unused;(void)unused;
    std::cout<<"C++: SAM histories/all bytes, hashes, mex products/field laws, certified subtraction, collision keys, tenure/aspiration and delta/full SA replay passed\n";
}
