#include <array>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "basic_utility/big_integer.hpp"
#include "basic_utility/fast_io.hpp"
#include "number_theory_basic/gcd_lcm.hpp"
#include "range_query/polynomial_add_sum.hpp"
#include "linear_algebra/matrix_product.hpp"
using cp::BigInt;
void check(bool ok) { if (!ok) throw std::runtime_error("BigInt check failed"); }
template <class E, class F> void throws(F f) {
    bool caught = false; try { f(); } catch (const E&) { caught = true; } check(caught);
}
template <class T> void bounds() {
    const auto lo = std::numeric_limits<T>::lowest(), hi = std::numeric_limits<T>::max();
    check(BigInt(lo).template to_integral<T>() == lo);
    check(BigInt(hi).template to_integral<T>() == hi);
    throws<std::out_of_range>([&] { (BigInt(lo)-1).template to_integral<T>(); });
    throws<std::out_of_range>([&] { (BigInt(hi)+1).template to_integral<T>(); });
}
int main() {
    bounds<signed char>(); bounds<unsigned char>(); bounds<short>(); bounds<unsigned short>();
    bounds<int>(); bounds<unsigned>(); bounds<long>(); bounds<unsigned long>();
    bounds<long long>(); bounds<unsigned long long>(); bounds<__int128_t>(); bounds<__uint128_t>();
    for (int a=-128;a<=127;++a) for (int b=-128;b<=127;++b) {
        const BigInt x=a,y=b;
        check(x+y==a+b && x-y==a-b && x*y==a*b);
        check((x<y)==(a<b) && (x<=y)==(a<=b) && (x>y)==(a>b) && (x>=y)==(a>=b));
        check((x==y)==(a==b) && (x!=y)==(a!=b) && bool(x)==bool(a));
        check(cp::gcd(x,y)==std::gcd(a,b));
        check(cp::lcm(x,y)==std::lcm(a,b));
        if(b) {
            const auto [q,r]=cp::divmod(x,y);
            check(q==a/b && r==a%b && x/y==q && x%y==r);
            check(x==q*y+r && cp::abs(r)<cp::abs(y));
        }
    }
    check(cp::gcd(12LL,18LL)==6); // Existing fixed-width overload remains available.
    check(BigInt("000000009")==9 && BigInt("+000")==0 && BigInt("-0").sign()==0);
    check(BigInt("-0XfF",16)==-255 && BigInt("FF",16).to_string(16)=="ff");
    for(const auto* token:{"","+","-"," 1","1 ","1_0","1.0","0x10","--1","12a","\xff"})
        throws<std::invalid_argument>([&]{BigInt::parse(token);});
    for(const auto* token:{"","0x","+0X","-0x","0x-1","0xfg","f f","-+1"})
        throws<std::invalid_argument>([&]{BigInt::parse(token,16);});
    throws<std::invalid_argument>([]{BigInt::parse("1",2);});
    throws<std::invalid_argument>([]{BigInt(1).to_string(8);});
    throws<std::invalid_argument>([]{BigInt::parse(std::string("12\0" "3",4));});
    BigInt unchanged=42;
    throws<std::domain_error>([&]{unchanged/=0;});check(unchanged==42);
    throws<std::domain_error>([&]{unchanged%=0;});check(unchanged==42);
    throws<std::domain_error>([]{cp::divmod(BigInt(1),BigInt(0));});
    for(const auto& text:std::array<std::string,5>{"0","1","-1",std::string(2000,'9'),"-"+std::string(2000,'8')}) {
        const BigInt a(text);
        check(BigInt(a.to_string())==a && BigInt(a.to_string(16),16)==a);
        auto x=a; x+=x; check(x==2*a);
        x=a; x-=x;check(x==0 && x.to_string(16)=="0");
        x=a; x*=x;check(x==a*a);
        if(a){x=a;x/=x;check(x==1);x=a;x%=x;check(x==0);}
        x=a;check(x++==a && x==a+1 && x--==a+1 && x==a);
        check(++x==a+1 && --x==a && +x==a && -(-x)==a);
        auto copy=a;copy+=1;check(a==BigInt(text));
        auto moved=std::move(copy);copy=7;check(moved==a+1 && copy==7);
        check(BigInt(a.native())==a);
    }
    std::istringstream input("+0009 -0 -255 12x 77");BigInt value=100;
    input>>value;check(value==9);input>>value;check(value==0);input>>value;check(value==-255);
    input>>value;check(input.fail() && value==-255);input.clear();input>>value;check(value==77);
    input>>value;check(input.fail() && input.eof() && value==77);
    std::ostringstream output;output<<std::hex<<std::showpos<<BigInt(-255);check(output.str()=="-255");
    std::istringstream hex("0xff");hex>>std::hex>>value;check(hex.fail() && value==77);
    std::istringstream exceptional("bad");exceptional.exceptions(std::ios::failbit);
    throws<std::ios_base::failure>([&]{exceptional>>value;});check(value==77);
    auto* file=std::tmpfile();check(file!=nullptr);
    {
        cp::BasicFastPrinter<3> printer(file);printer.writeln(BigInt("-12345678901234567890").to_string());printer.flush();
        std::rewind(file);cp::BasicFastScanner<2> scanner(file);std::string token;
        check(scanner.read(token) && BigInt::parse(token)==BigInt("-12345678901234567890"));
    }
    std::fclose(file);
    const BigInt huge("10000000000000000000000000000000000000000");
    using Action=cp::PolynomialAddSumAction<BigInt,2>;
    cp::ActedMonoidLazySegtree<Action> tree(Action::values({huge,huge,huge}));
    tree.apply(0,3,{huge,0,huge});check(tree.all_prod().sum==11*huge);
    cp::Matrix<BigInt> matrix{{huge,1},{0,huge}};
    auto square=cp::matrix_multiply(matrix,matrix);
    check(square(0,0)==huge*huge && square(0,1)==2*huge);
    BigInt factorial=1;for(int i=1;i<=100;++i)factorial*=i;
    check(factorial.to_string().size()==158 && factorial%BigInt("1000000000000000000000000")==0);
    std::cout<<"65536 small pairs, conversion bounds, parsing, aliases, streams and integration passed\n";
}
