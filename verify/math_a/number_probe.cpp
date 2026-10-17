#include <iostream>
#include <string>
#include <vector>
#include "number_theory_basic/all.hpp"
int main(){std::string op;while(std::cin>>op){try{if(op=="floor"){long long n,m,a,b;std::cin>>n>>m>>a>>b;std::cout<<cp::floor_sum(n,m,a,b)<<'\n';}else{int n;std::cin>>n;std::vector<long long> r(n),m(n);for(int i=0;i<n;++i)std::cin>>r[i]>>m[i];if(op=="crt"){auto result=cp::crt(r,m);if(result)std::cout<<result->remainder<<' '<<result->modulus<<'\n';else std::cout<<"none\n";}else{long long target;std::cin>>target;std::cout<<cp::garner(r,m,target)<<'\n';}}}catch(const std::overflow_error&){std::cout<<"overflow\n";}}}
