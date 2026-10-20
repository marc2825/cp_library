#include <iostream>
#include "basic_utility/meet_in_the_middle.hpp"
int main() {
    cp::MeetInTheMiddle search({3, -2, 5, 1});
    auto best = search.best_leq(6);
    std::cout << static_cast<long long>(best->sum) << ' ' << best->mask << ' '
              << search.count_equal(6) << '\n';
}
