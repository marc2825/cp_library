#pragma once
#include "../static_query/succinct_bit_vector.hpp"
#include "burrows_wheeler.hpp"
#include "detail/bwt_rank.hpp"
#include <numeric>

namespace cp {
// Packed BWT rank plus suffix positions sampled by text coordinate. No full SA/text retained.
class FMIndex {
    int size_ = 0, sample_rate_ = 32;
    std::array<int, 257> cumulative_{};
    string_algorithm_detail::BWTRank bwt_;
    SuccinctBitVector marked_;
    std::vector<int> samples_;
    int resolve(int row) const {
        int steps = 0;
        while (!marked_.get(row)) {
            const int c = bwt_.access(row);
            row = cumulative_[c] + bwt_.count(0, row, c);
            ++steps;
        }
        return (samples_[marked_.rank(row)] + steps) % (size_ + 1);
    }

  public:
    explicit FMIndex(std::string_view text = {}, int sample_rate = 32) : sample_rate_(sample_rate) {
        if (sample_rate <= 0)
            throw std::invalid_argument("FM sample rate must be positive");
        auto b = burrows_wheeler_transform(text);
        size_ = static_cast<int>(text.size());
        std::vector<int> last(b.last.begin(), b.last.end());
        bwt_ = string_algorithm_detail::BWTRank(last);
        std::array<int, 257> count{};
        for (int c : last)
            ++count[c];
        for (int c = 1; c <= 256; ++c)
            cumulative_[c] = cumulative_[c - 1] + count[c - 1];
        marked_ = SuccinctBitVector(size_ + 1);
        std::vector<std::pair<int, int>> selected;
        int row = b.primary, position = 0;
        for (int i = 0; i <= size_; ++i) {
            if (position % sample_rate_ == 0) {
                marked_.set(row);
                selected.emplace_back(row, position);
            }
            const int c = last[row];
            row = cumulative_[c] + bwt_.count(0, row, c);
            position = position ? position - 1 : size_;
        }
        marked_.build();
        std::sort(selected.begin(), selected.end());
        for (const auto &x : selected)
            samples_.push_back(x.second);
    }
    int size() const { return size_; }
    int sample_rate() const { return sample_rate_; }
    std::pair<int, int> interval(std::string_view pattern) const {
        int l = 0, r = size_ + 1;
        for (std::size_t i = pattern.size(); i-- > 0 && l < r;) {
            const int c = static_cast<unsigned char>(pattern[i]) + 1;
            l = cumulative_[c] + bwt_.count(0, l, c);
            r = cumulative_[c] + bwt_.count(0, r, c);
        }
        return {l, r};
    }
    int count(std::string_view pattern) const {
        auto p = interval(pattern);
        return p.second - p.first;
    }
    std::vector<int> locate(std::string_view pattern) const {
        auto p = interval(pattern);
        std::vector<int> out;
        for (int row = p.first; row < p.second; ++row)
            out.push_back(resolve(row));
        std::sort(out.begin(), out.end());
        return out;
    }
};
} // namespace cp
