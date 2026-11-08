#pragma once
#include "../convolution/multidimensional_convolution.hpp"
#include "polynomial.hpp"
namespace cp {
// Quotient F[x_0,...]/(x_0^shape[0],...). Row-major rectangular storage, positive axes.
template <int Mod = 998244353, int Root = 3> class MultivariateFPS {
    using F = fps_detail::Field<Mod>;
    std::vector<std::size_t> shape_, degree_;
    std::size_t volume_ = 1, total_ = 0;
    void check(const std::vector<int> &a) const {
        if (a.size() != volume_)
            throw std::invalid_argument("multivariate FPS storage mismatch");
    }

  public:
    explicit MultivariateFPS(std::vector<std::size_t> shape) : shape_(std::move(shape)) {
        for (auto x : shape_) {
            if (!x)
                throw std::invalid_argument("FPS axes must be positive");
            if (volume_ > std::vector<int>().max_size() / x)
                throw std::length_error("FPS volume overflow");
            volume_ *= x;
            total_ += x - 1;
        }
        degree_.resize(volume_);
        for (std::size_t i = 0; i < volume_; ++i) {
            auto k = i;
            for (std::size_t j = shape_.size(); j-- > 0;) {
                degree_[i] += k % shape_[j];
                k /= shape_[j];
            }
        }
    }
    const auto &shape() const { return shape_; }
    std::size_t size() const { return volume_; }
    std::vector<int> multiply(const std::vector<int> &a, const std::vector<int> &b) const {
        check(a);
        check(b);
        auto full = multidimensional_convolution<Mod, Root>(a, shape_, b, shape_);
        std::vector<int> out(volume_);
        for (std::size_t i = 0; i < volume_; ++i) {
            auto k = i;
            std::size_t source = 0, stride = 1;
            for (std::size_t j = shape_.size(); j-- > 0;) {
                source += (k % shape_[j]) * stride;
                k /= shape_[j];
                stride *= full.shape[j];
            }
            out[i] = full.coefficients[source];
        }
        return out;
    }
    std::vector<int> inverse(const std::vector<int> &a) const {
        check(a);
        std::vector<int> g(volume_);
        g[0] = F::inv(F::norm(a[0]));
        for (std::size_t precision = 1; precision <= total_;) {
            auto correction = multiply(a, g);
            for (auto &x : correction)
                x = F::sub(0, x);
            correction[0] = F::add(correction[0], 2 % Mod);
            g = multiply(g, correction);
            if (precision > total_ / 2)
                break;
            precision *= 2;
        }
        return g;
    }
    std::vector<int> logarithm(const std::vector<int> &a) const {
        check(a);
        if (F::norm(a[0]) != 1)
            throw std::domain_error("multivariate log needs constant one");
        if (total_ >= static_cast<std::size_t>(Mod))
            throw std::domain_error("total degree must be below characteristic");
        auto inv = fps_detail::inverses<Mod>(total_);
        std::vector<int> derivative(volume_);
        for (std::size_t i = 1; i < volume_; ++i)
            derivative[i] = F::mul(F::norm(a[i]), static_cast<int>(degree_[i]));
        auto result = multiply(derivative, inverse(a));
        for (std::size_t i = 1; i < volume_; ++i)
            result[i] = F::mul(result[i], inv[degree_[i]]);
        return result;
    }
    std::vector<int> exponential(const std::vector<int> &a) const {
        check(a);
        if (F::norm(a[0]))
            throw std::domain_error("multivariate exp needs constant zero");
        if (total_ >= static_cast<std::size_t>(Mod))
            throw std::domain_error("total degree must be below characteristic");
        std::vector<int> g(volume_);
        g[0] = 1;
        for (std::size_t precision = 1; precision <= total_;) {
            auto correction = polynomial_subtract<Mod>(a, logarithm(g));
            correction[0] = 1;
            g = multiply(g, correction);
            if (precision > total_ / 2)
                break;
            precision *= 2;
        }
        return g;
    }
    std::vector<int> power(std::vector<int> a, std::uint64_t exponent) const {
        check(a);
        std::vector<int> r(volume_);
        r[0] = 1;
        for (; exponent; exponent >>= 1) {
            if (exponent & 1)
                r = multiply(r, a);
            if (exponent > 1)
                a = multiply(a, a);
        }
        return r;
    }
};
} // namespace cp
