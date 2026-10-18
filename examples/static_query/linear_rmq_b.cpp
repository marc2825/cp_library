#include <iostream>
#include "static_query/linear_rmq.hpp"
int main() {
    cp::LinearRMQ<int> s({4,1,1,3});
    std::cout << s.argmin(0,4) << "\n";
}
