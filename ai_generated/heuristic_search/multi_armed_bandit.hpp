#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include "random.hpp"
namespace cp {
class MultiArmedBandit {
    std::vector<std::uint64_t> pulls_;std::vector<long double> means_;std::uint64_t total_=0;long double exploration_;
    std::size_t greedy()const{std::size_t best=0;for(std::size_t i=1;i<means_.size();++i)if(means_[i]>means_[best])best=i;return best;}
public:
    explicit MultiArmedBandit(std::size_t arms,long double exploration=1.4142135623730950488L):pulls_(arms),means_(arms),exploration_(exploration){if(!arms||!std::isfinite(exploration)||exploration<0)throw std::invalid_argument("invalid bandit parameters");}
    std::size_t size()const{return pulls_.size();}
    const std::vector<std::uint64_t>& pulls()const{return pulls_;}
    const std::vector<long double>& means()const{return means_;}
    std::uint64_t total_pulls()const{return total_;}
    void update(std::size_t arm,long double reward){if(arm>=size())throw std::out_of_range("bandit arm");if(!std::isfinite(reward)||reward<0||reward>1)throw std::invalid_argument("bandit rewards must be in [0,1]");if(total_==std::numeric_limits<std::uint64_t>::max())throw std::overflow_error("bandit pull count");++pulls_[arm];++total_;means_[arm]+=(reward-means_[arm])/static_cast<long double>(pulls_[arm]);}
    std::size_t select_ucb1()const{
        for(std::size_t i=0;i<size();++i)if(!pulls_[i])return i;
        // 大きい探索係数では全indexを同じ正数で割り、積のoverflowを避ける。
        const auto value=[&](std::size_t i){const auto bonus=std::sqrt(std::log(static_cast<long double>(total_))/static_cast<long double>(pulls_[i]));return exploration_>1?means_[i]/exploration_+bonus:means_[i]+exploration_*bonus;};
        std::size_t best=0;for(std::size_t i=1;i<size();++i)if(value(i)>value(best))best=i;return best;
    }
    std::size_t select_epsilon_greedy(Random&random,long double epsilon)const{
        if(!std::isfinite(epsilon)||epsilon<0||epsilon>1)throw std::invalid_argument("epsilon must be in [0,1]");
        for(std::size_t i=0;i<size();++i)if(!pulls_[i])return i;
        return random.real()<epsilon?std::size_t(random.uniform(size())):greedy();
    }
};
} // namespace cp
