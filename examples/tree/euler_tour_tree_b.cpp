#include <iostream>
#include "tree/euler_tour_tree.hpp"
long long sum(long long a,long long b){return a+b;}
long long identity(){return 0;}
int main() {
    cp::EulerTourTree<long long,sum,identity> t(std::vector<long long>{1,2,3});
    t.link(0,1); t.link(1,2); std::cout << t.component_prod(2) << ' ';
    t.cut(0,1); std::cout << t.component_prod(2) << '\n';
}
