#pragma once
#include "../wavelet_matrix.hpp"
#include <stdexcept>

namespace cp {
namespace seaweed_detail {
// Seaweed start labels ordered by end coordinate. Shared dominance representation for LCS/LIS.
class RankIndex {
    WaveletMatrix<int> starts_;

  public:
    explicit RankIndex(const std::vector<int> &starts = {}) : starts_(starts) {}
    int size() const { return starts_.size(); }
    int count(int end, int minimum_start) const {
        if (end < 0 || end > size())
            throw std::out_of_range("seaweed boundary outside index");
        return end - starts_.count_less(0, end, minimum_start);
    }
    int score(int left, int right) const { return right - left - count(right, left); }
};
} // namespace seaweed_detail
} // namespace cp
