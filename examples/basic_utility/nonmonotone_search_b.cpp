#include <iostream>
#include "basic_utility/nonmonotone_search.hpp"
int main() {
    auto best=cp::nonmonotone_argmin<int>(0,8,[](auto i){ return int((i^3)*2); },[](auto,auto){ return 0; });
    std::cout << best->first << "\n";
}
