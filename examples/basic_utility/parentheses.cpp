#include <iostream>
#include "basic_utility/parentheses.hpp"
int main() {
    cp::ParenthesesTable table(3);
    auto text = table.unrank(3);
    auto tree = cp::parentheses_to_tree(text);
    std::cout << table.count() << ' ' << text << ' ' << table.rank(text) << '\n';
    std::cout << tree.size() << ' ' << cp::tree_to_parentheses(tree) << '\n';
}
