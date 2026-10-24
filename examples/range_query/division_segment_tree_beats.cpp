#include <iostream>
#include "../../range_query/division_segment_tree_beats.hpp"
int main(){
    cp::DivisionSegmentTreeBeats tree({10,21,32,43});
    tree.range_divide(0,4,3); tree.range_add(1,3,2);
    std::cout<<tree.range_sum(0,4)<<' '<<tree.range_min(0,4)<<' '<<tree.range_max(0,4)<<'\n';
}
