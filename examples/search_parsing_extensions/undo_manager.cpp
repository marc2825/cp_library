#include "heuristic_search/undo_manager.hpp"
#include <iostream>
#include <numeric>
using namespace cp;
int main() {
    std::vector<int> values{1, 2};
    UndoManager history;
    auto before = history.snapshot();
    history.transaction([&](auto &log) {
        log.set(values[0], 3);
        log.set(values[1], 4);
    });
    auto score = [](const auto &s) { return std::accumulate(s.begin(), s.end(), 0); };
    std::cout << score(values) << " " << audit_score(values, 7, score).consistent << " ";
    history.rollback(before);
    std::cout << score(values) << "\n";
}
