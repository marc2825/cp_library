#pragma once
#include "detail/lce.hpp"
#include <set>
#include <string>

namespace cp {
struct LZ77Factor {
    int source = -1; // -1 is a literal; otherwise an absolute earlier position (overlap allowed).
    int length = 1;
    unsigned char literal = 0;
};
// Greedy longest previous factors, with self-references allowed. O(n log n) time and space.
inline std::vector<LZ77Factor> lempel_ziv_factorization(std::string_view text) {
    std::vector<int> bytes;
    for (unsigned char c : text)
        bytes.push_back(c);
    const string_algorithm_detail::LCEIndex index(bytes);
    std::set<int> previous;
    std::vector<LZ77Factor> out;
    for (int i = 0; i < index.n;) {
        const auto it = previous.lower_bound(index.rank[i]);
        int source = -1, length = 0;
        auto consider = [&](int rank) {
            const int p = index.sa[rank], k = index.lce(p, i);
            if (k > length || (k == length && k && p < source)) {
                source = p;
                length = k;
            }
        };
        if (it != previous.end())
            consider(*it);
        if (it != previous.begin())
            consider(*std::prev(it));
        if (!length)
            out.push_back({-1, 1, static_cast<unsigned char>(text[i])});
        else
            out.push_back({source, length, 0});
        const int end = i + std::max(1, length);
        while (i < end)
            previous.insert(index.rank[i++]);
    }
    return out;
}
inline std::string restore_lz77(const std::vector<LZ77Factor> &factors, std::size_t max_length = 100000000) {
    std::string out;
    for (const auto &f : factors) {
        if (f.length <= 0 || f.source < -1 || (f.source == -1 && f.length != 1) ||
            (f.source >= 0 && static_cast<std::size_t>(f.source) >= out.size()))
            throw std::invalid_argument("invalid LZ77 factor");
        if (static_cast<std::size_t>(f.length) > max_length - out.size())
            throw std::length_error("LZ77 output budget exceeded");
        if (f.source == -1)
            out.push_back(static_cast<char>(f.literal));
        else
            for (int k = 0; k < f.length; ++k)
                out.push_back(out[static_cast<std::size_t>(f.source) + k]);
    }
    return out;
}
} // namespace cp
