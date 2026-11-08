#include <iostream>
#include "dp_optimization/larsch.hpp"
int main() {
    using E=std::pair<bool,long long>;
    cp::Larsch<E> s(3,[](int i,int j)->E{if(j>i) return {true,0}; return {false,1LL*(i-j)*(i-j)};});
    for(int i=0;i<3;++i) { if(i) std::cout << ' '; std::cout << s.next_argmin(); }
    std::cout << '\n';
}
