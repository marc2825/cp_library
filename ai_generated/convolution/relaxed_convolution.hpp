#pragma once
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../fps/detail/common.hpp"

namespace cp {
// append(a[t],b[t]) -> sum_{i=0}^t a[i]b[t-i]。総計O(N log^2 N)。
// 事前指定した項数まで。docs/convolution/relaxed_convolution.md
template <int Mod = 998244353, int Root = 3> class RelaxedConvolution {
    using F = fps_detail::Field<Mod>;
    std::size_t limit_;
    std::vector<int> a_, b_, sums_;
    void transfer(std::size_t first, const std::vector<int>& a, const std::vector<int>& b) {
        const auto product = fps_detail::multiply<Mod, Root>(a, b);
        for (std::size_t j = 0; j < product.size() && j < limit_ - first; ++j)
            sums_[first + j] = F::add(sums_[first + j], product[j]);
    }
public:
    explicit RelaxedConvolution(std::size_t terms) : limit_(terms) {
        fps_detail::check_precision<Mod, Root>(terms);
        sums_.resize(terms); a_.reserve(terms); b_.reserve(terms);
    }
    std::size_t size() const { return a_.size(); }
    std::size_t capacity() const { return limit_; }
    int append(int a, int b) {
        if (size() == limit_) throw std::length_error("relaxed convolution is full");
        const auto t = size();
        a_.push_back(F::norm(a)); b_.push_back(F::norm(b));
        int value = F::add(sums_[t], F::mul(a_[t], b_[0]));
        if (t) value = F::add(value, F::mul(a_[0], b_[t]));
        const auto count = t + 1;
        // min(i,j) in [block,2*block)。各添字対をちょうど一度予約する。
        for (std::size_t block = 1; block <= count / 2 && count % block == 0; block *= 2) {
            if (count == limit_) break;
            const std::vector<int> x(a_.begin() + count - block, a_.end());
            const std::vector<int> y(b_.begin() + block, b_.begin() + 2 * block);
            transfer(count, x, y);
            if (count != 2 * block) {
                const std::vector<int> u(a_.begin() + block, a_.begin() + 2 * block);
                const std::vector<int> v(b_.begin() + count - block, b_.end());
                transfer(count, u, v);
            }
        }
        return value;
    }
};
} // namespace cp
