#pragma once
#include <cstddef>
#include <stdexcept>
#include "../convolution/relaxed_convolution.hpp"

namespace cp {
namespace fps_detail {
template <int Mod, int Root, bool Exponential> class OnlineFPS {
    using F = Field<Mod>;
    RelaxedConvolution<Mod, Root> product_;
    std::size_t limit_, size_ = 0;
    int previous_ = 0, constant_inverse_ = 0;
public:
    explicit OnlineFPS(std::size_t terms) : product_(terms ? terms - 1 : 0), limit_(terms) {
        check_precision<Mod, Root>(terms);
        if (Exponential && terms > static_cast<std::size_t>(Mod)) throw std::domain_error("online exp indices must be invertible");
    }
    std::size_t size() const { return size_; }
    int append(int coefficient) {
        if (size_ == limit_) throw std::length_error("online FPS is full");
        coefficient = F::norm(coefficient);
        if (!size_) {
            if constexpr (Exponential) {
                if (coefficient) throw std::domain_error("online exp needs zero constant");
                previous_ = 1;
            } else { constant_inverse_ = F::inv(coefficient); previous_ = constant_inverse_; }
        } else {
            int input = coefficient;
            if constexpr (Exponential) input = F::mul(input, static_cast<int>(size_ % Mod));
            const int sum = product_.append(input, previous_);
            if constexpr (Exponential) previous_ = F::mul(sum, F::inv(static_cast<int>(size_)));
            else previous_ = F::sub(0, F::mul(sum, constant_inverse_));
        }
        ++size_; return previous_;
    }
};
} // namespace fps_detail
// a[t]を追加し、inv(a)[t] / exp(a)[t]を即時に返す。
template <int Mod = 998244353, int Root = 3> using OnlineFPSInverse = fps_detail::OnlineFPS<Mod, Root, false>;
template <int Mod = 998244353, int Root = 3> using OnlineFPSExp = fps_detail::OnlineFPS<Mod, Root, true>;
} // namespace cp
