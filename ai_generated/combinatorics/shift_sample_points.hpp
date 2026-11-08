#pragma once
// 説明: docs/combinatorics/shift_sample_points.md
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../fps/interpolation.hpp"
#include "../fps/multipoint_evaluation.hpp"
namespace cp {
// f(0)..f(n-1)からf(first)..f(first+count-1)。deg f<n、n<=Mod。
template<int Mod=998244353,int Root=3> std::vector<int> shift_sample_points(const std::vector<int>& samples,long long first,std::size_t count){
    using Field=fps_detail::Field<Mod>;if(samples.size()>static_cast<std::size_t>(Mod))throw std::domain_error("sample points are not distinct modulo modulus");
    fps_detail::check_precision<Mod,Root>(samples.size());fps_detail::check_precision<Mod,Root>(count);if(!count)return {};if(samples.empty())return std::vector<int>(count,0);std::vector<int> xs(samples.size());for(std::size_t i=0;i<xs.size();++i)xs[i]=static_cast<int>(i);auto coefficients=polynomial_interpolation<Mod,Root>(xs,samples);std::vector<int> targets(count);int x=Field::norm(first);for(auto& point:targets){point=x;if(++x==Mod)x=0;}return multipoint_evaluation<Mod,Root>(coefficients,targets);
}
} // namespace cp
