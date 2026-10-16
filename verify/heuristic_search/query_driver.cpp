// Local deterministic protocol, not an online judge submission.
#include <cstdint>
#include <iostream>
#include <vector>
#include "heuristic_search/all.hpp"
int main() {
    char kind;
    while (std::cin >> kind) {
        if (kind=='R'||kind=='U'||kind=='I'||kind=='X') {
            std::uint64_t seed;std::size_t count;std::cin>>seed;
            cp::Random random(seed);
            if(kind=='R'){std::cin>>count;for(std::size_t i=0;i<count;++i)std::cout<<random()<<' ';}
            if(kind=='U'){std::uint64_t bound;std::cin>>bound>>count;for(std::size_t i=0;i<count;++i)std::cout<<random.uniform(bound)<<' ';}
            if(kind=='I'){std::int64_t lo,hi;std::cin>>lo>>hi>>count;for(std::size_t i=0;i<count;++i)std::cout<<random.integer(lo,hi)<<' ';}
            if(kind=='X'){std::uint64_t lo,hi;std::cin>>lo>>hi>>count;for(std::size_t i=0;i<count;++i)std::cout<<random.integer(lo,hi)<<' ';}
        } else {
            int n,start,goal;std::cin>>n>>start>>goal;
            if(kind=='S') {
                cp::AnnealingOptions options;int cooling;std::uint64_t seed;
                std::cin>>options.max_iterations>>options.start_temperature>>options.end_temperature>>cooling>>seed;
                options.goal=goal?cp::SearchGoal::maximize:cp::SearchGoal::minimize;
                options.cooling=cooling?cp::CoolingSchedule::linear:cp::CoolingSchedule::geometric;
                std::vector<long long> score(n);for(auto&x:score)std::cin>>x;
                std::vector<int> proposals(options.max_iterations);for(auto&x:proposals)std::cin>>x;
                std::size_t index=0;cp::Random random(seed);
                const auto r=cp::simulated_annealing(start,[&](int v){return score[v];},[&](int,cp::Random&){return proposals[index++];},random,options);
                std::cout<<r.best_state<<' '<<r.current_state<<' '<<r.iterations<<' '<<r.evaluations<<' '<<r.accepted<<' '<<static_cast<int>(r.termination);
            } else {
                std::size_t width_or_mode,cap;std::cin>>width_or_mode>>cap;
                std::vector<long long> score(n);for(auto&x:score)std::cin>>x;
                std::vector<std::vector<int>> graph(n);for(auto&row:graph){std::size_t size;std::cin>>size;row.resize(size);for(auto&v:row)std::cin>>v;}
                auto evaluate=[&](int v){return score[v];};auto expand=[&](int v){return graph[v];};
                if(kind=='L') {
                    cp::LocalSearchOptions options;options.goal=goal?cp::SearchGoal::maximize:cp::SearchGoal::minimize;options.max_iterations=cap;
                    options.mode=width_or_mode?cp::LocalSearchMode::best_improvement:cp::LocalSearchMode::first_improvement;
                    const auto r=cp::local_search(start,evaluate,expand,options);
                    std::cout<<r.best_state<<' '<<r.iterations<<' '<<r.evaluations<<' '<<r.accepted<<' '<<static_cast<int>(r.termination);
                } else {
                    cp::BeamSearchOptions options;options.goal=goal?cp::SearchGoal::maximize:cp::SearchGoal::minimize;options.width=width_or_mode;options.max_depth=cap;
                    const auto r=cp::beam_search(start,evaluate,expand,options);
                    std::cout<<r.depth<<' '<<r.evaluations<<' '<<static_cast<int>(r.termination)<<' '<<r.frontier.size();for(const auto&e:r.frontier)std::cout<<' '<<e.state;
                }
            }
        }
        std::cout<<'\n';
    }
}
