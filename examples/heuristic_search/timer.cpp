#include <iostream>
#include "heuristic_search/timer.hpp"
int main() {
    cp::Timer timer(1.9); // Seconds, measured from construction by steady_clock.
    std::cout << timer.limit() << '\n';
    // while (!timer.expired()) { ... }  // Cooperative wall-time budget.
    timer.reset(0);
    std::cout << timer.expired() << ' ' << timer.progress() << '\n';
}
