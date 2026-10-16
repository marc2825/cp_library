#pragma once
#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>

// First true state t in [0,updates]; updates+1 if never true.
// reset(): initial state; apply(i): apply update i; check(q): monotone predicate.
// "Parallel" batches queries, not threads. check must preserve the represented state.
// Docs: docs/offline_query/parallel_binary_search.md
namespace cp {

template <class Reset, class Apply, class Check>
std::vector<int> parallel_binary_search(int updates, int queries,
                                        Reset reset, Apply apply, Check check) {
    assert(0 <= updates && updates < std::numeric_limits<int>::max());
    assert(0 <= queries);
    std::vector<int> low(queries, -1), high(queries, updates + 1);
    // Linked buckets avoid a vector allocation for each time state in each round.
    std::vector<int> head, next(queries);
    if (queries == 0) return high;
    head.resize(updates + 1);
    while (true) {
        std::fill(head.begin(), head.end(), -1);
        int last = -1;
        for (int q = 0; q < queries; ++q) {
            if (static_cast<long long>(high[q]) - low[q] <= 1) continue;
            int mid = low[q] + int((static_cast<long long>(high[q]) - low[q]) / 2);
            next[q] = head[mid];
            head[mid] = q;
            last = std::max(last, mid);
        }
        if (last < 0) return high;
        reset();
        for (int t = 0; t <= last; ++t) {
            if (t > 0) apply(t - 1);
            for (int q = head[t]; q >= 0; q = next[q]) {
                if (check(q)) high[q] = t;
                else low[q] = t;
            }
        }
    }
}

}  // namespace cp
