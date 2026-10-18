#pragma once
#include <algorithm>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../linear_recurrence/detail/common.hpp"
#include "../heuristic_search/random.hpp"

namespace cp {
using LinearMatroidPairs=std::vector<std::pair<std::vector<int>,std::vector<int>>>;
namespace matroid_parity_detail {
template <int Mod> int rank(std::vector<std::vector<int>> a,std::size_t columns){
    using F=linear_recurrence_detail::Arithmetic<Mod>;int r=0;
    for(std::size_t col=0;col<columns&&static_cast<std::size_t>(r)<a.size();++col){std::size_t row=r;while(row<a.size()&&!a[row][col])++row;if(row==a.size())continue;std::swap(a[r],a[row]);const int inverse=F::inverse(a[r][col]);for(std::size_t j=col;j<columns;++j)a[r][j]=F::mul(a[r][j],inverse);for(std::size_t i=r+1;i<a.size();++i){const int f=a[i][col];for(std::size_t j=col;j<columns;++j)a[i][j]=F::sub(a[i][j],F::mul(f,a[r][j]));}++r;}return r;
}
template <int Mod> LinearMatroidPairs normalize(LinearMatroidPairs pairs){
    static_assert(linear_recurrence_detail::prime(Mod),"linear matroid parity needs a prime field");
    if(pairs.size()>static_cast<std::size_t>(INT_MAX/2))throw std::length_error("too many matroid pairs");
    const auto d=pairs.empty()?0:pairs[0].first.size();if(d>static_cast<std::size_t>(INT_MAX))throw std::length_error("matroid dimension exceeds int");
    for(auto&pair:pairs){if(pair.first.size()!=d||pair.second.size()!=d)throw std::invalid_argument("matroid pair dimensions differ");pair.first=linear_recurrence_detail::normalize<Mod>(pair.first);pair.second=linear_recurrence_detail::normalize<Mod>(pair.second);}return pairs;
}
template <int Mod> bool independent(const LinearMatroidPairs&pairs,const std::vector<int>&selected){
    std::vector<std::vector<int>> columns;for(int i:selected){columns.push_back(pairs[i].first);columns.push_back(pairs[i].second);}return rank<Mod>(std::move(columns),pairs.empty()?0:pairs[0].first.size())==int(2*selected.size());
}
}
// 小規模向け決定的厳密版。O(2^m poly(d,m))、結果は選んだpair添字。
template <int Mod=998244353> std::vector<int> linear_matroid_parity_exact(const LinearMatroidPairs&input){
    const auto pairs=matroid_parity_detail::normalize<Mod>(input);std::vector<int> best,current;
    std::function<void(std::size_t)> dfs=[&](std::size_t i){if(current.size()+pairs.size()-i<=best.size())return;if(i==pairs.size()){best=current;return;}current.push_back(int(i));if(matroid_parity_detail::independent<Mod>(pairs,current))dfs(i+1);current.pop_back();dfs(i+1);};dfs(0);return best;
}
// Lovaszの交代行列のrankを使う確率的版。返す集合の独立性は必ず検査。
// 最大性はMonte Carlo。固定seed、有限回の試行。O(trials*m*d^3)の保守的上界。
template <int Mod=998244353> std::vector<int> linear_matroid_parity(const LinearMatroidPairs&input,std::size_t trials=4,std::uint64_t seed=20261018){
    using F=linear_recurrence_detail::Arithmetic<Mod>;const auto pairs=matroid_parity_detail::normalize<Mod>(input);if(!trials)throw std::invalid_argument("matroid parity needs a positive trial count");
    const auto d=pairs.empty()?0:pairs[0].first.size();std::vector<int> best;
    // 全試行が不運でも実行可能な貪欲集合を保持。
    for(std::size_t i=0;i<pairs.size();++i){auto next=best;next.push_back(int(i));if(matroid_parity_detail::independent<Mod>(pairs,next))best=std::move(next);}
    Random random(seed);
    for(std::size_t trial=0;trial<trials;++trial){
        std::vector<int> coefficients(pairs.size());for(auto&x:coefficients)x=int(random.uniform(Mod));
        std::vector<std::vector<int>> matrix(d,std::vector<int>(d));
        auto update=[&](std::size_t k,int sign){const auto&[u,v]=pairs[k];for(std::size_t i=0;i<d;++i)for(std::size_t j=0;j<d;++j){const int delta=F::mul(coefficients[k],F::sub(F::mul(u[i],v[j]),F::mul(v[i],u[j])));matrix[i][j]=sign>0?F::add(matrix[i][j],delta):F::sub(matrix[i][j],delta);}};
        for(std::size_t k=0;k<pairs.size();++k)update(k,1);
        const int target=matroid_parity_detail::rank<Mod>(matrix,d);std::vector<int> selected;
        for(std::size_t k=0;k<pairs.size();++k){update(k,-1);if(matroid_parity_detail::rank<Mod>(matrix,d)!=target){update(k,1);selected.push_back(int(k));}}
        if(selected.size()==static_cast<std::size_t>(target/2)&&selected.size()>best.size()&&matroid_parity_detail::independent<Mod>(pairs,selected))best=std::move(selected);
    }
    return best;
}
} // namespace cp
