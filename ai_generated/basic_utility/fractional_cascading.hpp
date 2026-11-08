#pragma once
// 説明: docs/basic_utility/fractional_cascading.md
#include <algorithm>
#include <cassert>
#include <iterator>
#include <vector>

namespace cp {
// ソート済み列を連鎖させる。全列のlower_boundをO(log N + 列数)。
template<class T> class FractionalCascading {
    struct Catalog { std::vector<T> values; std::vector<int> original, next; };
    std::vector<Catalog> catalogs_;
public:
    explicit FractionalCascading(const std::vector<std::vector<T>>& lists = {}) : catalogs_(lists.size()) {
        for (int i = int(lists.size()) - 1; i >= 0; --i) {
            assert(std::is_sorted(lists[i].begin(), lists[i].end()));
            auto& c = catalogs_[i];
            std::vector<T> sample;
            if (i + 1 < int(lists.size())) {
                const auto& next = catalogs_[i+1].values;
                for (std::size_t j = 1; j < next.size(); j += 2) sample.push_back(next[j]);
            }
            std::merge(lists[i].begin(), lists[i].end(), sample.begin(), sample.end(), std::back_inserter(c.values));
            int p = 0, q = 0;
            for (const T& x : c.values) {
                while (p < int(lists[i].size()) && lists[i][p] < x) ++p;
                c.original.push_back(p);
                if (i + 1 < int(lists.size())) {
                    const auto& next = catalogs_[i+1].values;
                    while (q < int(next.size()) && next[q] < x) ++q;
                }
                c.next.push_back(q);
            }
            c.original.push_back(int(lists[i].size()));
            c.next.push_back(i+1 < int(lists.size()) ? int(catalogs_[i+1].values.size()) : 0);
        }
    }
    std::vector<int> lower_bounds(const T& x) const {
        std::vector<int> result;
        if (catalogs_.empty()) return result;
        int p = int(std::lower_bound(catalogs_[0].values.begin(), catalogs_[0].values.end(), x) - catalogs_[0].values.begin());
        for (std::size_t i = 0; i < catalogs_.size(); ++i) {
            const auto& c = catalogs_[i];
            result.push_back(c.original[p]);
            if (i + 1 < catalogs_.size()) {
                p = c.next[p];
                const auto& next = catalogs_[i+1].values;
                // 一つ前のsampleと現在のsampleの間には未抽出要素が高々一つ。
                if (p > 0 && !(next[p-1] < x)) --p;
            }
        }
        return result;
    }
};
} // namespace cp
