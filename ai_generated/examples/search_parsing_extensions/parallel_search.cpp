#include "heuristic_search/parallel_search.hpp"
#include <iostream>
using namespace cp;
struct Run {
    int best_state, best_score;
};
int main() {
    auto task = [](std::size_t, Random &random, const std::atomic<bool> &) {
        Run result{0, 1000};
        for (int i = 0; i < 3; ++i) {
            int x = int(random.uniform(100)), score = std::abs(x - 37);
            if (score < result.best_score)
                result = {x, score};
        }
        return result;
    };
    auto result = parallel_multi_start(4, task, 42);
    std::cout << result.completed << " " << *result.best_run_index << " " << result.best_run().best_state
              << " " << result.best_run().best_score << "\n";
}
