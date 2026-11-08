#pragma once
#include "border_tree.hpp"
#include "z_algorithm.hpp"

namespace cp {
// Exact incremental Z queries via the online border tree; earlier Z values can grow after append.
class OnlineZ {
    BorderTree borders_;

  public:
    explicit OnlineZ(std::string_view text = {}) : borders_(text) {}
    void append(unsigned char c) { borders_.append(c); }
    void append(std::string_view text) { borders_.append(text); }
    int size() const { return borders_.size(); }
    int query(int position) const {
        string_algorithm_detail::range(size(), position, position);
        int l = 0, r = size() - position;
        while (l < r) {
            const int m = l + (r - l + 1) / 2;
            if (borders_.is_border(m, position + m))
                l = m;
            else
                r = m - 1;
        }
        return l;
    }
    std::vector<int> values() const { return z_algorithm(borders_.text()); }
};
} // namespace cp
