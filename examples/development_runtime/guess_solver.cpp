#include <iostream>
#include "basic_utility/debug.hpp"
int main() {
    int upper;
    if (!(std::cin >> upper) || upper < 1) return 1;
    int low = 1, high = upper;
    while (low <= high) {
        const int mid = low + (high - low) / 2;
        CP_DEBUG(low, high, mid);
        std::cout << "? " << mid << std::endl;
        int reply;
        if (!(std::cin >> reply)) return 1;
        if (reply == 0) { std::cout << "! " << mid << std::endl; return 0; }
        if (reply < 0) high = mid - 1;
        else low = mid + 1;
    }
    return 1;
}
