#include <iostream>
#include "offline_query/rollback_mo.hpp"
int main() {
    std::vector<int> a{2,3,5},history; int sum=0; cp::RollbackMo mo(3); mo.add_query(0,2); mo.add_query(1,3);
    auto add=[&](int p){history.push_back(p);sum+=a[p];};
    mo.run([&]{history.clear();sum=0;},add,add,[&]{return history.size();},
        [&](std::size_t token){while(history.size()>token){sum-=a[history.back()];history.pop_back();}},
        [&](int id){std::cout<<id<<":"<<sum<<" ";});
    std::cout<<"\n";
}
