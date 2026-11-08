#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "search_common.hpp"
namespace cp {
template <class T>std::vector<std::vector<T>> parameter_grid(const std::vector<std::vector<T>>&axes){
    std::vector<std::vector<T>> result(1);
    for(const auto&axis:axes){if(axis.empty())return {};if(result.size()>result.max_size()/axis.size())throw std::length_error("parameter grid too large");std::vector<std::vector<T>> next;next.reserve(result.size()*axis.size());for(const auto&prefix:result)for(const auto&value:axis){auto row=prefix;row.push_back(value);next.push_back(std::move(row));}result=std::move(next);}return result;
}
template <class Score>struct ParameterTuningResult {std::size_t best_index;std::vector<long double> mean_scores;std::vector<std::vector<Score>> scores;};
// 全設定で同じseed列を使用。evaluate(config,seed)は有限なスコアを返す。
template <class Config,class Evaluate>auto tune_parameters(const std::vector<Config>&configs,const std::vector<std::uint64_t>&seeds,Evaluate evaluate,SearchGoal goal=SearchGoal::minimize){
    using Score=std::decay_t<std::invoke_result_t<Evaluate&,const Config&,std::uint64_t>>;
    heuristic_detail::validate_goal(goal);if(configs.empty()||seeds.empty())throw std::invalid_argument("tuning needs configurations and seeds");
    ParameterTuningResult<Score> result{0,{}, {}};
    for(const auto&config:configs){std::vector<Score> scores;long double mean=0;for(std::size_t i=0;i<seeds.size();++i){const Score value=evaluate(config,seeds[i]);heuristic_detail::validate_score(value);scores.push_back(value);mean+=(static_cast<long double>(value)-mean)/static_cast<long double>(i+1);if(!std::isfinite(mean))throw std::overflow_error("parameter tuning mean is nonfinite");}result.scores.push_back(std::move(scores));result.mean_scores.push_back(mean);const auto index=result.mean_scores.size()-1;if(heuristic_detail::better(mean,result.mean_scores[result.best_index],goal))result.best_index=index;}
    return result;
}
} // namespace cp
