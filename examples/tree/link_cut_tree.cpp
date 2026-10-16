#include <iostream>
#include <string>
#include "tree/link_cut_tree.hpp"
std::string op(std::string a, std::string b) { return a + b; }
std::string e() { return {}; }
int main() {
    cp::LinkCutTree<std::string, op, e> t(std::vector<std::string>{"a", "b", "c"});
    t.link(0, 1); t.link(1, 2); std::cout << *t.path_prod(2, 0) << ' ';
    t.cut(1, 2); std::cout << t.connected(0, 2) << '\n';
}
