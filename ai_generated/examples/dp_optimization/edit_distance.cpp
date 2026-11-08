#include <iostream>
#include <string>
#include "dp_optimization/edit_distance.hpp"
int main() {
    std::string a = "kitten", b = "sitting";
    auto result = cp::edit_alignment(a, b);
    std::cout << result.distance << '\n';
    for (auto step : result.steps) {
        if (step.kind == cp::EditKind::Keep) std::cout << a[step.source_index];
        else if (step.kind != cp::EditKind::Erase) std::cout << b[step.target_index];
    }
    std::cout << '\n';
}
