#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include "string_algorithm/all.hpp"
#include "game_theory/all.hpp"
#include "heuristic_search/all.hpp"

std::string bytes() {
    std::string hex, s; std::cin >> hex;
    if (hex == "-") return s;
    for (std::size_t i = 0; i < hex.size(); i += 2) s.push_back(static_cast<char>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    return s;
}
template <class T> std::vector<T> read(std::size_t n) { std::vector<T> a(n); for (auto& x : a) std::cin >> x; return a; }
template <class T> void print(const std::vector<T>& a) { std::cout << a.size(); for (auto x : a) std::cout << ' ' << x; std::cout << '\n'; }
template <class R> void search_result(const R& r) {
    std::cout << r.best_state << ' ' << r.current_state << ' ' << r.best_score << ' ' << r.current_score << ' '
              << r.iterations << ' ' << r.evaluations << ' ' << r.accepted << ' ' << static_cast<int>(r.termination) << '\n';
}
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr); char op;
    while (std::cin >> op) {
        if (op == 'S') {
            auto s = bytes(); std::size_t queries; std::cin >> queries; cp::SuffixAutomaton a;
            for (unsigned char c : s) { a.append(c); (void)a.count_occurrences(std::string(1, static_cast<char>(c))); }
            std::cout << a.distinct_substrings() << ' ' << a.state_count();
            for (std::size_t i = 0; i < queries; ++i) { const auto p = bytes(); const auto first = a.first_occurrence(p); std::cout << ' ' << a.contains(p) << ' ' << a.count_occurrences(p) << ' ' << (first ? *first : -1); }
            std::cout << '\n';
        } else if (op == 'L') {
            const auto s = bytes(); auto f = cp::lyndon_factorization(s); std::cout << f.size(); for (auto [l,r] : f) std::cout << ' ' << l << ' ' << r; std::cout << '\n';
        } else if (op == 'C') {
            const auto a = bytes(), b = bytes(); const auto r = cp::longest_common_substring(a,b); std::cout << r.first_left << ' ' << r.second_left << ' ' << r.length << '\n';
        } else if (op == 'H') {
            std::size_t n; std::uint32_t row_base, col_base; int top,bottom,left,right; std::cin >> n >> row_base >> col_base;
            std::vector<std::string> grid(n); for (auto& row : grid) row = bytes(); std::cin >> top >> bottom >> left >> right;
            const auto h = cp::RollingHash2D(grid,row_base,col_base).get(top,bottom,left,right); std::cout << h[0] << ' ' << h[1] << '\n';
        } else if (op == 'N') {
            std::uint64_t a,b; std::cin >> a >> b; std::cout << cp::nim_product(a,b) << '\n';
        } else if (op == 'P') {
            std::size_t n,repeats,cap; std::cin >> n >> repeats >> cap; const auto a=read<int>(n); const auto p=cp::detect_grundy_period(a,repeats,cap);
            if (p) std::cout << p->preperiod << ' ' << p->period << '\n'; else std::cout << "-1\n";
        } else if (op == 'G') {
            std::size_t n,terms;std::uint64_t index;std::cin>>n>>terms>>index;const auto r=cp::subtraction_grundy(read<int>(n),terms);
            std::cout<<(r.period?static_cast<long long>(r.period->preperiod):-1)<<' '<<(r.period?r.period->period:0)<<' ';
            try { std::cout<<r.value(index); } catch(const std::out_of_range&) { std::cout<<-1; }
            std::cout<<' '<<r.values.size();for(int x:r.values)std::cout<<' '<<x;std::cout<<'\n';
        } else if (op == 'Z') {
            std::size_t n,k,changes;std::uint64_t seed;std::cin>>n>>k>>seed>>changes;auto state=read<int>(n);cp::ZobristHash z(n,k,seed);auto h=z.hash(state);std::cout<<h;
            for(std::size_t i=0;i<changes;++i){std::size_t p,v;std::cin>>p>>v;h=z.change(h,p,state[p],v);state[p]=static_cast<int>(v);std::cout<<' '<<h;}std::cout<<'\n';
        } else if (op == 'B' || op == 'T') {
            std::size_t n,start,budget,parameter;int goal;std::cin>>n>>start>>goal>>parameter>>budget;auto scores=read<long long>(n);auto keys=read<int>(n);std::vector<std::vector<int>> graph(n);
            for(auto& row:graph){std::size_t m;std::cin>>m;row=read<int>(m);}
            const auto evaluate=[&](int x){return scores[x];};const auto expand=[&](int x){return graph[x];};const auto key=[&](int x){return keys[x];};
            if(op=='B'){cp::BeamSearchOptions o;o.width=parameter;o.max_depth=budget;o.goal=goal?cp::SearchGoal::maximize:cp::SearchGoal::minimize;auto r=cp::beam_search_unique(static_cast<int>(start),evaluate,expand,key,o);std::cout<<r.depth<<' '<<r.evaluations<<' '<<static_cast<int>(r.termination)<<' '<<r.frontier.size();for(const auto& e:r.frontier)std::cout<<' '<<e.state<<' '<<e.score;std::cout<<'\n';}
            else{cp::TabuSearchOptions o;o.tenure=parameter;o.max_iterations=budget;o.goal=goal?cp::SearchGoal::maximize:cp::SearchGoal::minimize;search_result(cp::tabu_search(static_cast<int>(start),evaluate,expand,key,o));}
        } else if (op == 'D') {
            long long state,target;std::size_t n;std::cin>>state>>target>>n;long long score=(state-target)*(state-target);
            for(std::size_t i=0;i<n;++i){long long move;int accept;std::cin>>move>>accept;
                const auto delta=[&](const long long& x,const long long& d)->long long{return 2*(x-target)*d+d*d;};
                cp::try_scored_move(state,score,move,delta,[](long long& x,long long d){x+=d;},[](long long& x,long long d){x-=d;},[&](const long long&,long long,long long){return bool(accept);});
                std::cout<<state<<' '<<score<<' ';
            }std::cout<<'\n';
        }
    }
}
