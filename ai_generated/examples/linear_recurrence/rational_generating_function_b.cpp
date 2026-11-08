#include <iostream>
#include <vector>
#include "../../linear_recurrence/rational_generating_function.hpp"

int main() {
    cp::RationalGeneratingFunction<> a({1}, {1, -1, -1});
    std::cout << a.coefficient(6) << "\n";
}
