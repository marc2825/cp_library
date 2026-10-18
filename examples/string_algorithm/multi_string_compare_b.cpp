#include <iostream>
#include <vector>
#include "../../string_algorithm/multi_string_compare.hpp"

int main() {
    cp::MultiStringCompare strings({"banana", "band"});
    std::cout << strings.lcp(0, 0, 6, 1, 0, 4) << " " << strings.compare(0, 0, 6, 1, 0, 4) << "\n";
}
