#pragma once
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>
#include "burnside.hpp"

namespace cp {
// 全群要素の置換を入力。要素は検査するが群の閉性/重複/完全性は利用者の責任。
class CycleIndex {
    int degree_=0;
    long long order_=0;
    std::map<std::vector<int>,long long> terms_;
public:
    explicit CycleIndex(const std::vector<std::vector<int>>& group) {
        if (group.empty() || group.size()>static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::invalid_argument("invalid group size");
        if (group[0].size()>static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::invalid_argument("permutation too large");
        degree_=static_cast<int>(group[0].size()); order_=static_cast<long long>(group.size());
        for (const auto& p:group) {
            if (p.size()!=static_cast<std::size_t>(degree_)) throw std::invalid_argument("permutation degree mismatch");
            std::vector<bool> used(degree_);
            for (int v:p) { if(v<0 || v>=degree_ || used[v]) throw std::invalid_argument("invalid permutation"); used[v]=true; }
            used.assign(degree_,false); std::vector<int> counts(static_cast<std::size_t>(degree_)+1);
            for (int v=0;v<degree_;++v) if(!used[v]) {
                int length=0,u=v; do { used[u]=true; ++length; u=p[u]; } while(u!=v);
                ++counts[length];
            }
            ++terms_[counts];
        }
    }
    int degree() const { return degree_; }
    long long group_order() const { return order_; }
    const std::map<std::vector<int>,long long>& terms() const { return terms_; }
    // weights[i]を長さiのcycleに代入。色の重みw_cならweights[i]=sum_c w_c^i。
    long long evaluate(const std::vector<long long>& weights,long long modulus=998244353) const {
        if (weights.size()!=static_cast<std::size_t>(degree_)+1 || modulus<=0) throw std::invalid_argument("invalid cycle weights or modulus");
        long long sum=0;
        for (const auto& entry:terms_) {
            auto value=entry.second%modulus;
            for (int i=1;i<=degree_;++i) if(entry.first[i]) value=mod_multiply(value,mod_pow(weights[i],entry.first[i],modulus),modulus);
            sum=combinatorics_detail::add_mod(sum,value,modulus);
        }
        return burnside_average(sum,order_,modulus);
    }
    long long colorings(long long colors,long long modulus=998244353) const {
        if (colors<0) throw std::invalid_argument("negative color count");
        return evaluate(std::vector<long long>(static_cast<std::size_t>(degree_)+1,colors),modulus);
    }
};
} // namespace cp
