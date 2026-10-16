#include "static_query/monotonic_queue.hpp"
#include "static_query/sliding_window_minimum.hpp"
#include <iostream>
#include <vector>
int main() {
    std::vector<int> a{4, 2, 2, 5, 1};
    for (int x : cp::sliding_window_minimum(a, 3)) std::cout << x << ' '; // 2 2 1
    std::cout << '\n';
    for (int i : cp::sliding_window_argmin(a, 3)) std::cout << i << ' '; // 1 1 4
    std::cout << '\n';
    cp::MonotonicQueue<int> queue;
    queue.push(4); queue.push(2); queue.push(2);
    queue.pop(); // Removes the logical 4, even though 4 is no longer a candidate.
    std::cout << queue.size() << ' ' << queue.best() << ' ' << queue.best_index() << '\n'; // 2 2 1
}
