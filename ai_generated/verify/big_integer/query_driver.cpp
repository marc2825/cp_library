#include <iostream>
#include <string>
#include "basic_utility/big_integer.hpp"
int main() {
    char operation;std::string a,b;
    while(std::cin>>operation>>a) {
        if(operation=='H') {
            auto x=cp::BigInt::parse(a,16);std::cout<<x<<' '<<x.to_string(16)<<'\n';continue;
        }
        if(operation=='D') {
            auto x=cp::BigInt::parse(a);std::cout<<x<<' '<<x.to_string(16)<<'\n';continue;
        }
        std::cin>>b;cp::BigInt x(a),y(b);
        if(operation=='A')std::cout<<x+y<<'\n';
        else if(operation=='S')std::cout<<x-y<<'\n';
        else if(operation=='M')std::cout<<x*y<<'\n';
        else if(operation=='G')std::cout<<cp::gcd(x,y)<<'\n';
        else if(operation=='L')std::cout<<cp::lcm(x,y)<<'\n';
        else if(operation=='Q'){auto [q,r]=cp::divmod(x,y);std::cout<<q<<' '<<r<<'\n';}
        else if(operation=='C')std::cout<<(x<y)<<' '<<(x==y)<<' '<<(x>y)<<'\n';
        else return 1;
    }
}
