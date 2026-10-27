#pragma once
#include "../fps/detail/common.hpp"
namespace cp {
// Fixed-kernel linear map x -> (kernel*x)[0:output_size], and its matrix transpose.
// Kernel spectrum is shared by forward/transpose calls; coefficients use ordinary dot products.
template <int Mod = 998244353, int Root = 3> class TransposedConvolutionPlan {
    using F = fps_detail::Field<Mod>;
    std::size_t input_, output_, size_;
    std::vector<int> spectrum_;

  public:
    TransposedConvolutionPlan(const std::vector<int> &kernel, std::size_t input_size, std::size_t output_size)
        : input_(input_size), output_(output_size) {
        if (kernel.empty() || !input_)
            throw std::invalid_argument("linear convolution plan needs nonempty kernel and input");
        const auto shape = convolution_detail::shape(kernel.size(), input_, ntt_max_size<Mod, Root>());
        if (output_ > shape.first)
            throw std::invalid_argument("output exceeds full convolution size");
        size_ = shape.second;
        spectrum_ = fps_detail::prefix<Mod>(kernel, size_);
        ntt<Mod, Root>(spectrum_);
    }
    std::size_t input_size() const { return input_; }
    std::size_t output_size() const { return output_; }
    std::vector<int> apply(const std::vector<int> &x) const {
        if (x.size() != input_)
            throw std::invalid_argument("linear map input size mismatch");
        auto y = fps_detail::prefix<Mod>(x, size_);
        ntt<Mod, Root>(y);
        for (std::size_t i = 0; i < size_; ++i)
            y[i] = F::mul(y[i], spectrum_[i]);
        ntt<Mod, Root>(y, true);
        y.resize(output_);
        return y;
    }
    std::vector<int> transpose(const std::vector<int> &weights) const {
        if (weights.size() != output_)
            throw std::invalid_argument("transpose input size mismatch");
        auto y = fps_detail::prefix<Mod>(weights, size_);
        ntt<Mod, Root>(y);
        for (std::size_t i = 0; i < size_; ++i)
            y[i] = F::mul(y[i], spectrum_[(size_ - i) % size_]);
        ntt<Mod, Root>(y, true);
        y.resize(input_);
        return y;
    }
};
} // namespace cp
