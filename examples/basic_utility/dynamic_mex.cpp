#include <iostream>
#include "basic_utility/dynamic_mex.hpp"
int main() {
    cp::DynamicMex values;
    values.insert(0); values.insert(1); values.insert(1);
    values.erase(1);
    std::cout << values.mex() << ' ' << values.count(1) << '\n';
    values.insert(3); values.erase(0);
    std::cout << values.mex() << ' ' << values.size() << '\n';
}
