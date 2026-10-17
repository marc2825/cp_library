#include <iostream>
#include "union_find/merge_tree.hpp"
int main() {
    cp::MergeTree d(3); int a=d.merge(0,1), b=d.merge(1,2);
    std::cout<<a<<" "<<b<<" "<<d.parent[0]<<"\n";
}
