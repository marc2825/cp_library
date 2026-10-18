#include <iostream>
#include "offline_query/rectangle_add_point_get.hpp"
int main() {
    cp::RectangleAddPointGet<> s({{0,0,3,2}}); s.add(0,0,3,2,4);
    std::cout << s.get(2,1) << " " << s.get(3,1) << "\n";
}
