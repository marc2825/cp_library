#include <iostream>
#include "offline_query/hilbert_order_mo.hpp"
int main() {
    cp::HilbertOrderMo m(4); m.add_query(1,3); int total=0,answer=0;
    m.run([&](int i){total+=i;},[&](int i){total-=i;},[&](int){answer=total;});
    std::cout << answer << "\n";
}
