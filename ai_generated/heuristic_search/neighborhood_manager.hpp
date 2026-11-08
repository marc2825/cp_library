#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
#include "simulated_annealing.hpp"
namespace cp {
class NeighborhoodManager {
public:
    struct Statistics {std::uint64_t attempts=0,accepted=0,improved=0;};
private:
    std::vector<long double> weights_,sums_;std::vector<Statistics> statistics_;std::size_t base_=1;
    void check(std::size_t i)const{if(i>=size())throw std::out_of_range("neighborhood ID");}
public:
    explicit NeighborhoodManager(const std::vector<long double>&weights):weights_(weights.size()),statistics_(weights.size()){
        if(weights.empty())throw std::invalid_argument("neighborhood manager needs at least one entry");
        while(base_<weights.size()){if(base_>sums_.max_size()/4)throw std::length_error("too many neighborhoods");base_*=2;}
        sums_.resize(2*base_);for(std::size_t i=0;i<weights.size();++i)set_weight(i,weights[i]);
    }
    std::size_t size()const{return weights_.size();}
    long double total_weight()const{return sums_[1];}
    const std::vector<long double>& weights()const{return weights_;}
    const std::vector<Statistics>& statistics()const{return statistics_;}
    void set_weight(std::size_t id,long double weight){
        check(id);if(!std::isfinite(weight)||weight<0)throw std::invalid_argument("neighborhood weight must be finite and nonnegative");
        std::vector<std::pair<std::size_t,long double>> updates;auto node=base_+id;long double value=weight;updates.emplace_back(node,value);
        while(node>1){value+=sums_[node^1];if(!std::isfinite(value))throw std::overflow_error("neighborhood total weight");node/=2;updates.emplace_back(node,value);}
        for(auto[index,sum]:updates)sums_[index]=sum;
        weights_[id]=weight;
    }
    std::size_t select(Random&random)const{
        const auto total=total_weight();if(total<=0)throw std::domain_error("all neighborhoods are disabled");
        long double target=std::min(static_cast<long double>(random.real())*total,std::nextafter(total,0.L));std::size_t node=1;
        while(node<base_){node*=2;if(target>=sums_[node]){target-=sums_[node];++node;}target=std::min(target,std::nextafter(sums_[node],0.L));}
        return node-base_;
    }
    void observe(std::size_t id,bool accepted,bool improved){check(id);if(improved&&!accepted)throw std::invalid_argument("an improving move must be accepted");auto&s=statistics_[id];if(s.attempts==std::numeric_limits<std::uint64_t>::max())throw std::overflow_error("neighborhood attempt count");++s.attempts;if(accepted)++s.accepted;if(improved)++s.improved;}
    void adapt_weights(long double prior=1,long double floor=0.01L){if(!std::isfinite(prior)||prior<=0||!std::isfinite(floor)||floor<0||floor>1)throw std::invalid_argument("invalid neighborhood adaptation");for(std::size_t i=0;i<size();++i)if(weights_[i]>0){const auto&s=statistics_[i];const auto score=(static_cast<long double>(s.improved)/2+prior/2)/(static_cast<long double>(s.attempts)/2+prior);set_weight(i,std::max(floor,score));}}
};
// propose(current,neighborhood_id,rng)->candidate。採否と最良更新をmanagerへ記録。
template <class State,class Evaluate,class Propose>
auto managed_simulated_annealing(State initial,Evaluate evaluate,Propose propose,NeighborhoodManager&manager,Random&random,const AnnealingOptions&options={})
    ->SearchResult<State,heuristic_detail::ScoreOf<Evaluate,State>>{
    using Score=heuristic_detail::ScoreOf<Evaluate,State>;heuristic_detail::validate_goal(options.goal);(void)annealing_temperature(options,0);
    const Score score=evaluate(std::as_const(initial));heuristic_detail::validate_score(score);SearchResult<State,Score> result{initial,initial,score,score};
    for(std::size_t step=0;step<options.max_iterations;++step){
        if(options.stop&&options.stop()){result.termination=SearchTermination::stopped;break;}
        if(manager.total_weight()<=0){result.termination=SearchTermination::exhausted;break;}
        const auto id=manager.select(random);State candidate=propose(std::as_const(result.current_state),id,random);heuristic_detail::count_evaluation(result.evaluations);
        const Score next=evaluate(std::as_const(candidate));heuristic_detail::validate_score(next);
        const auto progress=options.progress?options.progress():(options.max_iterations<=1?0.L:static_cast<long double>(step)/static_cast<long double>(options.max_iterations-1));
        const auto temperature=annealing_temperature(options,progress);bool accept=!heuristic_detail::better(result.current_score,next,options.goal);
        if(!accept)accept=random.real()<std::exp(-heuristic_detail::worsening_loss(result.current_score,next,options.goal)/temperature);
        const bool improved=accept&&heuristic_detail::better(next,result.best_score,options.goal);manager.observe(id,accept,improved);
        if(accept){result.current_state=std::move(candidate);result.current_score=next;++result.accepted;if(improved){result.best_state=result.current_state;result.best_score=next;}}
        ++result.iterations;
    }
    return result;
}
} // namespace cp
