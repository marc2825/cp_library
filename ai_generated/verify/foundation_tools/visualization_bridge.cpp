#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include "basic_utility/big_rational.hpp"
#include "basic_utility/arena.hpp"
#include "basic_utility/bit_operations.hpp"
#include "basic_utility/grid.hpp"
#include "basic_utility/rollback_array.hpp"
#include "basic_utility/radix_sort.hpp"
struct Node{int value;std::size_t previous;};
int main(){
    std::cout<<"big_rational {\"value\":\""<<cp::BigRational::parse("6/-8")+cp::BigRational(1,6)<<"\"}\n";
    cp::Arena<Node,2> nodes;nodes.emplace(Node{10,std::numeric_limits<std::size_t>::max()});nodes.emplace(Node{20,0});nodes.emplace(Node{30,0});
    std::cout<<"arena {\"nodes\":[";
    for(std::size_t i=0;i<nodes.size();i++){if(i)std::cout<<',';std::cout<<"{\"id\":"<<i<<",\"block\":"<<i/2<<",\"slot\":"<<i%2<<",\"value\":"<<nodes[i].value<<",\"previous\":";if(i)std::cout<<nodes[i].previous;else std::cout<<"null";std::cout<<'}';}
    std::cout<<"],\"capacity\":"<<nodes.capacity()<<"}\n";
    std::cout<<"bit_operations {\"masks\":[";bool first=true;for(auto mask:cp::FixedPopcountMasks(5,2)){if(!first)std::cout<<',';first=false;std::cout<<'"'<<mask<<'"';}std::cout<<"],\"more\":false}\n";
    const std::vector<std::string> grid{"abc","def"};std::cout<<"grid {\"transformed\":[";first=true;
    for(const auto& row:cp::grid_rotate_clockwise(grid)){if(!first)std::cout<<',';first=false;std::cout<<'"'<<row<<'"';}
    std::cout<<"],\"neighbors\":[";first=true;for(auto [r,c]:cp::grid_shape(grid).neighbors4(0,1)){if(!first)std::cout<<',';first=false;std::cout<<'['<<r<<','<<c<<']';}std::cout<<"]}\n";
    cp::RollbackArray<int> values(std::vector<int>{10,20});auto root=values.snapshot();values.set(0,17);values.set(1,99);values.rollback(root);
    std::cout<<"rollback_array {\"values\":["<<values[0]<<','<<values[1]<<"],\"history\":"<<values.history_size()<<"}\n";
    std::vector<std::pair<short,int>> sorted{{3,0},{-1,1},{3,2},{0,3}};cp::radix_sort_by(sorted,[](const auto& value){return value.first;});
    std::cout<<"radix_sort {\"values\":[";first=true;for(auto [value,id]:sorted){(void)id;if(!first)std::cout<<',';first=false;std::cout<<'"'<<value<<'"';}
    std::cout<<"],\"indices\":[";first=true;for(auto [value,id]:sorted){(void)value;if(!first)std::cout<<',';first=false;std::cout<<id;}std::cout<<"]}\n";
}
