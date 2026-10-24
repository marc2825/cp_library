#include <iostream>
#include "../../ordered_structure/rbst_segment_tree.hpp"
#include <string>
std::string op(std::string a,std::string b){return a+b;}
std::string e(){return {};}
int main(){
    cp::RBSTSegmentTree<std::string,op,e> tree({"a","b","c","d"});
    tree.insert(2,"X"); tree.reverse(1,4);
    std::cout<<tree.all_prod()<<' '<<tree.prod(1,4)<<'\n';
    auto [left,right]=tree.split(3);
    std::cout<<right.concat(left).all_prod()<<'\n';
}
