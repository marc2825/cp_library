#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>
#include "simulated_annealing.hpp"
namespace cp {
template <class State,class Score>struct MultiStartAnnealingResult {
    std::vector<SearchResult<State,Score>> runs;
    std::size_t best_run_index=0;
    const SearchResult<State,Score>& best_run()const{return runs[best_run_index];}
};
// factory(run_index,run_rng)->独立した初期状態。各runに独立seedを割り当てる。
template <class Factory,class Evaluate,class Propose>
auto multi_start_sa(std::size_t starts,Factory factory,Evaluate evaluate,Propose propose,std::uint64_t seed=20261018,const AnnealingOptions&options={}){
    using State=std::decay_t<std::invoke_result_t<Factory&,std::size_t,Random&>>;using Score=heuristic_detail::ScoreOf<Evaluate,State>;
    if(!starts)throw std::invalid_argument("multi-start needs at least one run");
    MultiStartAnnealingResult<State,Score> result;Random seeds(seed);
    for(std::size_t run=0;run<starts;++run){
        if(run&&options.stop&&options.stop())break;
        Random random(seeds());auto initial=factory(run,random);result.runs.push_back(simulated_annealing(std::move(initial),evaluate,propose,random,options));
        if(heuristic_detail::better(result.runs.back().best_score,result.best_run().best_score,options.goal))result.best_run_index=run;
        if(result.runs.back().termination==SearchTermination::stopped)break;
    }
    return result;
}
} // namespace cp
