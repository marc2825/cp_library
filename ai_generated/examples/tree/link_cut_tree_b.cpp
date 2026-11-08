#include <iostream>
#include "tree/link_cut_tree.hpp"
long long sum(long long a,long long b){return a+b;}
long long identity(){return 0;}
int main() {
    cp::LinkCutTree<long long,sum,identity> t(std::vector<long long>{1,2,3});
    t.link(0,1); t.link(1,2); std::cout << *t.path_prod(0,2) << ' ' ;
    t.cut(1,2); std::cout << t.connected(0,2) << '\n';
}
