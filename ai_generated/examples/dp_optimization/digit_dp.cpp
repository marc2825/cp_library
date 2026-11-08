#include <iostream>
#include "dp_optimization/digit_dp.hpp"
int main() {
    // 1〜1000 のうち桁和が3の倍数。0はstarted=falseで除く。
    auto count = cp::digit_dp("1000", 3, 0,
        [](int state,int digit,bool,bool) { return (state+digit)%3; },
        [](int state,bool started) { return state==0 && started; });
    std::cout << count << '\n';
}
