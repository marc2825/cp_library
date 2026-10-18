#include <iostream>
#include <map>
#include <tuple>
#include <vector>
#include "basic_utility/hash_map.hpp"
#include "basic_utility/static_modint.hpp"
#include "graph/csr_graph.hpp"
#include "range_query/polynomial_add_sum.hpp"
struct ConstantHash { int operator()(long long) const { return 7; } };
int main(){
    std::cout<<"seeded_hash \""<<cp::SeededHash(42).hash64(std::make_tuple(-7LL,std::vector<long long>{1,2,3}))<<"\"\n";
    cp::HashMap<long long,long long,ConstantHash> table;
    table.insert(1,7);table.insert(9,3);table.erase(1);const auto query=*table.find(9);table.insert(17,8);
    std::map<long long,long long> ordered;table.for_each([&](auto k,auto v){ordered[k]=v;});
    std::cout<<"hash_map {\"entries\":[";bool first=true;
    for(auto [k,v]:ordered){if(!first)std::cout<<',';first=false;std::cout<<"[\""<<k<<"\",\""<<v<<"\"]";}
    std::cout<<"],\"queries\":[\""<<query<<"\"]}\n";
    cp::CSRGraph<int> graph(3,{{0,1,5},{1,1,2},{0,1,7}});
    std::cout<<"csr_graph {\"offsets\":[";first=true;
    for(auto x:graph.offsets()){if(!first)std::cout<<',';first=false;std::cout<<x;}
    std::cout<<"],\"arcs\":[";first=true;
    for(const auto&a:graph.arcs()){if(!first)std::cout<<',';first=false;std::cout<<'['<<a.from<<','<<a.to<<','<<a.cost<<','<<a.idx<<','<<a.rev<<']';}
    std::cout<<"]}\n";
    using M=cp::AffineFunctionMonoid<cp::StaticModInt<101>>;
    auto value=M::op({2,1},{3,4});
    std::cout<<"monoid_catalog {\"a\":\""<<value.a<<"\",\"b\":\""<<value.b<<"\",\"atX\":\""<<value(5)<<"\"}\n";
    using A=cp::PolynomialAddSumAction<long long,2>;
    cp::ActedMonoidLazySegtree<A> tree(A::values({1,2,3,4}));tree.apply(0,4,{0,0,1});
    std::cout<<"polynomial_add_sum {\"values\":[";
    for(int i=0;i<4;i++){if(i)std::cout<<',';std::cout<<'"'<<tree.get(i).sum<<'"';}
    std::cout<<"],\"sum\":\""<<tree.all_prod().sum<<"\"}\n";
}
