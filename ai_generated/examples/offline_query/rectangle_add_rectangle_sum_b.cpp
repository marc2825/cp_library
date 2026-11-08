#include <iostream>
#include "offline_query/rectangle_add_rectangle_sum.hpp"
int main() {
    cp::RectangleAddRectangleSum<> s({{0,0,3,2}}); s.add(0,0,3,2,4);
    std::cout << s.sum(1,0,3,2) << "\n";
}
