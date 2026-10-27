#pragma once
#include "fast_subset_convolution.hpp"
#include <limits>
#include <memory>
namespace cp {
namespace online_subset_detail {
template <int Mod> class Fixed {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    std::vector<int> kernel_, low_, cross_;
    std::size_t used_ = 0;
    std::unique_ptr<Fixed> child_;

  public:
    explicit Fixed(std::vector<int> kernel) : kernel_(std::move(kernel)) {}
    int push(int value) {
        value = F::norm(value);
        auto n = kernel_.size();
        if (n == 1) {
            ++used_;
            return F::mul(kernel_[0], value);
        }
        auto half = n / 2;
        if (!child_)
            child_ = std::make_unique<Fixed>(std::vector<int>(kernel_.begin(), kernel_.begin() + half));
        int answer = child_->push(value);
        if (used_ < half)
            low_.push_back(value);
        else
            answer = F::add(answer, cross_[used_ - half]);
        ++used_;
        if (used_ == half) {
            cross_ =
                subset_convolution_mod<Mod>(std::vector<int>(kernel_.begin() + half, kernel_.end()), low_);
            child_.reset();
            low_.clear();
        }
        return answer;
    }
};
template <int Mod> class Both {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    std::size_t size_, used_ = 0;
    std::vector<int> a_, b_;
    std::unique_ptr<Both> low_;
    std::unique_ptr<Fixed<Mod>> left_, right_;

  public:
    explicit Both(std::size_t size) : size_(size) {}
    int push(int a, int b) {
        a = F::norm(a);
        b = F::norm(b);
        if (size_ == 1) {
            ++used_;
            return F::mul(a, b);
        }
        int result;
        if (used_ < size_ / 2) {
            if (!low_)
                low_ = std::make_unique<Both>(size_ / 2);
            result = low_->push(a, b);
            a_.push_back(a);
            b_.push_back(b);
        } else
            result = F::add(left_->push(b), right_->push(a));
        ++used_;
        if (used_ == size_ / 2) {
            left_ = std::make_unique<Fixed<Mod>>(std::move(a_));
            right_ = std::make_unique<Fixed<Mod>>(std::move(b_));
            low_.reset();
        }
        return result;
    }
};
} // namespace online_subset_detail
// Append both functions in numeric mask order; returns their subset convolution at the new mask.
// Full stream O(b^3 2^b), O(b 2^b) working storage. Later input may depend on previous outputs.
template <int Mod = 998244353> class OnlineSubsetConvolution {
    std::size_t capacity_, used_ = 0;
    online_subset_detail::Both<Mod> engine_;
    static std::size_t capacity(std::size_t bits) {
        if (bits >= std::numeric_limits<std::size_t>::digits || bits > 24)
            throw std::length_error("online subset convolution supports at most 24 bits");
        return std::size_t(1) << bits;
    }

  public:
    explicit OnlineSubsetConvolution(std::size_t bits) : capacity_(capacity(bits)), engine_(capacity_) {}
    std::size_t size() const { return used_; }
    std::size_t max_size() const { return capacity_; }
    int push(int a, int b) {
        if (used_ == capacity_)
            throw std::length_error("online subset convolution stream is full");
        int answer = engine_.push(a, b);
        ++used_;
        return answer;
    }
};
} // namespace cp
