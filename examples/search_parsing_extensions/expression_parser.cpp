#include "parsing/expression_parser.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto tree = parse_expression("-2^2 + 3*(x+1)");
    std::cout << evaluate_integer_expression(tree, {{"x", 4}}) << "\n";
}
