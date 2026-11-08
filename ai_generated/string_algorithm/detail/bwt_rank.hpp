#pragma once
#include "../../static_query/succinct_bit_vector.hpp"
#include <array>
#include <vector>

namespace cp {
namespace string_algorithm_detail {
// Nine packed rank levels for the fixed alphabet 0..256; no n-word coordinate dictionary.
class BWTRank {
    std::array<SuccinctBitVector, 9> bits_;
    std::array<int, 9> zeros_{};

  public:
    BWTRank() = default;
    explicit BWTRank(std::vector<int> symbols) {
        const int n = static_cast<int>(symbols.size());
        std::vector<int> next(n);
        for (int d = 0; d < 9; ++d) {
            bits_[d] = SuccinctBitVector(n);
            int zero = 0;
            for (int i = 0; i < n; ++i) {
                if ((symbols[i] >> (8 - d)) & 1)
                    bits_[d].set(i);
                else
                    ++zero;
            }
            zeros_[d] = zero;
            bits_[d].build();
            int a = 0, b = zero;
            for (int c : symbols)
                next[((c >> (8 - d)) & 1) ? b++ : a++] = c;
            symbols.swap(next);
        }
    }
    int count(int l, int r, int c) const {
        for (int d = 0; d < 9; ++d) {
            const int a = bits_[d].rank(l), b = bits_[d].rank(r);
            if ((c >> (8 - d)) & 1) {
                l = zeros_[d] + a;
                r = zeros_[d] + b;
            } else {
                l -= a;
                r -= b;
            }
        }
        return r - l;
    }
    int access(int p) const {
        int c = 0;
        for (int d = 0; d < 9; ++d) {
            const bool bit = bits_[d].get(p);
            c = c * 2 + bit;
            const int ones = bits_[d].rank(p);
            p = bit ? zeros_[d] + ones : p - ones;
        }
        return c;
    }
};
} // namespace string_algorithm_detail
} // namespace cp
