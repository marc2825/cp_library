#include <chrono>
#include <iostream>
#include <random>
#include <string>
#include <boost/version.hpp>
#include "basic_utility/big_integer.hpp"
int main() {
    std::cout<<"Boost "<<BOOST_LIB_VERSION<<", "<<__VERSION__<<'\n';
    std::mt19937 random(111026);
    for(int digits:{1000,10000}) {
        std::string s(digits,'1'),t(digits,'1');
        for(int i=1;i<digits;++i){s[i]=char('0'+random()%10);t[i]=char('0'+random()%10);}
        using Clock=std::chrono::steady_clock;
        const auto start=Clock::now();cp::BigInt a(s),b(t);const auto parsed=Clock::now();
        cp::BigInt product;
        for(int i=0;i<10;++i)product=a*b;
        const auto multiplied=Clock::now();auto [q,r]=cp::divmod(product,a);const auto divided=Clock::now();
        auto g=cp::gcd(a,b);const auto gcd=Clock::now();
        const auto text=product.to_string();const auto formatted=Clock::now();
        if(q!=b || r!=0 || a%g!=0 || b%g!=0 || text.size()<std::size_t(2*digits-1))return 1;
        const auto ms=[](auto x,auto y){return std::chrono::duration<double,std::milli>(y-x).count();};
        std::cout<<digits<<" digits: parse2="<<ms(start,parsed)<<"ms multiply_mean="<<ms(parsed,multiplied)/10
            <<"ms divide="<<ms(multiplied,divided)<<"ms gcd="<<ms(divided,gcd)<<"ms format_product="<<ms(gcd,formatted)<<"ms\n";
    }
}
