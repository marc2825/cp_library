#include <iostream>
#include "../../ordered_structure/finger_tree.hpp"
#include <string>
std::string op(std::string a,std::string b){return a+b;}
std::string e(){return {};}
int main(){
    using Tree=cp::FingerTree<std::string,op,e>;
    Tree original({"a","b","c"}),tail({"x","y"});
    auto [left,right]=original.concat(tail).split(2);
    std::cout<<left.all_prod()<<' '<<right.all_prod()<<'\n';
    right.pop_front();
    std::cout<<right.all_prod()<<' '<<original.all_prod()<<'\n';
}
