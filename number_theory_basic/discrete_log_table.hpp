#pragma once
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <vector>
#include "detail/advanced_arithmetic.hpp"
#include "integer_kth_root.hpp"

namespace cp {
// 固定 base/modulus の BSGS を再利用。非可逆 base も gcd の列を共有。
class DiscreteLogTable {
    using U = std::uint64_t;
    struct Stage { U modulus, divisor, factor; };
    U modulus_, reduced_, width_ = 0, step_ = 0, inverse_factor_ = 0;
    std::vector<Stage> stages_;
    std::unordered_map<U, U> baby_;
public:
    DiscreteLogTable(U base, U modulus, U max_baby_steps = 2000000) : modulus_(modulus), reduced_(modulus) {
        if (!modulus) throw std::invalid_argument("zero discrete log modulus");
        base %= modulus;
        U factor = 1 % modulus;
        for (;;) {
            const U g = std::gcd(base, reduced_);
            stages_.push_back({reduced_, g, factor});
            if (g == 1) break;
            reduced_ /= g;
            factor = nt_detail::mul(factor % reduced_, (base / g) % reduced_, reduced_);
        }
        if (reduced_ == 1) return;
        width_ = integer_kth_root(reduced_, 2);
        if (width_ * width_ < reduced_) ++width_;
        if (width_ > max_baby_steps || width_ > baby_.max_size()) throw std::length_error("BSGS baby-step budget");
        baby_.reserve(static_cast<std::size_t>(width_));
        U value = 1;
        for (U i = 0; i < width_; ++i) { baby_.emplace(value, i); value = nt_detail::mul(value, base % reduced_, reduced_); }
        step_ = nt_detail::inverse(value, reduced_);
        inverse_factor_ = nt_detail::inverse(factor, reduced_);
    }
    U modulus() const { return modulus_; }
    std::size_t baby_steps() const { return baby_.size(); }
    std::optional<U> query(U target) const {
        target %= modulus_;
        U offset = 0;
        for (const auto& stage : stages_) {
            if (target == stage.factor) return offset;
            if (stage.divisor == 1) break;
            if (target % stage.divisor != 0) return std::nullopt;
            target /= stage.divisor; ++offset;
        }
        U current = nt_detail::mul(target, inverse_factor_, reduced_);
        for (U i = 0; i <= width_; ++i) {
            const auto found = baby_.find(current);
            if (found != baby_.end()) {
                const BigInt answer = BigInt(i) * width_ + found->second + offset;
                if (answer <= std::numeric_limits<U>::max()) return answer.to_integral<U>();
            }
            current = nt_detail::mul(current, step_, reduced_);
        }
        return std::nullopt;
    }
};
} // namespace cp
