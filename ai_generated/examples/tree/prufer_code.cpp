#include <iostream>
#include "../../tree/prufer_code.hpp"
int main() {
    auto tree = cp::prufer_decode(5, {3, 3, 1});
    auto code = cp::prufer_encode(tree);
    for (std::size_t i = 0; i < code.size(); ++i)
        std::cout << code[i] << (i + 1 == code.size() ? '\n' : ' ');
    std::cout << cp::prufer_decode(1, {}).size() << ' '
              << cp::prufer_decode(2, {}).edge_count() << '\n';
}
