#include <iostream>
#include <string>
#include <vector>
#include "basic_utility/big_rational.hpp"
#include "basic_utility/bit_operations.hpp"
#include "basic_utility/grid.hpp"
#include "basic_utility/radix_sort.hpp"
int main(){
    char kind;
    while(std::cin>>kind){
        if(kind=='R'){
            std::string sa,sb;std::cin>>sa>>sb;auto a=cp::BigRational::parse(sa),b=cp::BigRational::parse(sb);
            std::cout<<a+b<<' '<<a-b<<' '<<a*b<<' ';if(b)std::cout<<a/b;else std::cout<<"none";
            std::cout<<' '<<(a<b)<<' '<<(a==b)<<'\n';
        }else if(kind=='B'){
            std::uint64_t value;std::cin>>value;std::cout<<cp::topbit(value)<<' '<<cp::lowbit(value)<<' '<<cp::popcount(value)<<' ';
            auto next=cp::next_same_popcount(value);if(next)std::cout<<*next;else std::cout<<"none";std::cout<<'\n';
        }else if(kind=='S'){
            int n;std::cin>>n;std::vector<std::pair<__int128_t,int>> values;
            for(int i=0;i<n;i++){std::string s;std::cin>>s;values.emplace_back(cp::BigInt(s).to_integral<__int128_t>(),i);}
            cp::radix_sort_by(values,[](const auto& value){return value.first;});
            bool first=true;for(const auto& value:values){if(!first)std::cout<<' ';first=false;std::cout<<value.second;}std::cout<<'\n';
        }else return 1;
    }
}
