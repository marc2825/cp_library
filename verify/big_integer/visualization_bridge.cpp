#include <iostream>
#include "basic_utility/big_integer.hpp"
int main(){
    const cp::BigInt parsed("-0x123456789ABCDEF",16);
    std::cout<<"parse {\"decimal\":\""<<parsed<<"\",\"hex\":\""<<parsed.to_string(16)<<"\"}\n";
    std::cout<<"arithmetic {\"value\":\""<<cp::BigInt(999999999)*123456<<"\"}\n";
    auto [q,r]=cp::divmod(cp::BigInt("-12345678901234567890"),cp::BigInt(97));
    std::cout<<"division {\"quotient\":\""<<q<<"\",\"remainder\":\""<<r<<"\"}\n";
    std::cout<<"gcd {\"gcd\":\""<<cp::gcd(cp::BigInt("12345678901234567890"),cp::BigInt("9876543210"))<<"\"}\n";
}
