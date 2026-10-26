#include "common.hpp"
#include "../../number_theory_basic/min_mod_linear.hpp"
#include "../../number_theory_basic/mod_sum_polynomial.hpp"
using cp::BigInt;
long long floor_div(long long x,long long m){return x>=0?x/m:-((-x+m-1)/m);}
long long power(long long x,unsigned k){long long r=1;while(k--)r*=x;return r;}
int main(){
    auto op=[](const std::string&a,const std::string&b){return a+b;};
    for(int n=0;n<8;++n)for(int m=1;m<10;++m)for(int a=0;a<12;++a)for(int b=0;b<12;++b){std::string expected;int previous=0;
        for(int i=0;i<n;++i){int y=(a*i+b)/m;expected+=std::string(y-previous,'U');expected+='R';previous=y;}
        CHECK(cp::floor_word_product(n,m,a,b,std::string("U"),std::string("R"),std::string(),op)==expected);
    }
    for(int n=0;n<6;++n)for(int m=1;m<6;++m)for(int a=-4;a<=4;++a)for(int b=-4;b<=4;++b){
        const auto moments=cp::floor_sum_moments(n,m,a,b,3);
        for(unsigned p=0;p<=3;++p)for(unsigned q=0;p+q<=3;++q){long long expected=0;for(int i=0;i<n;++i)expected+=power(i,p)*power(floor_div(a*i+b,m),q);CHECK(moments[p][q]==expected);}
        CHECK(cp::floor_sum_big(n,m,a,b)==moments[0][1]);
    }
    std::mt19937 rng(281028);
    for(int test=0;test<2000;++test){const int l=int(rng()%15)-7,r=l+int(rng()%20),m=1+int(rng()%30),a=int(rng()%61)-30,b=int(rng()%61)-30;
        auto result=cp::min_mod_linear(l,r,a,b,m);if(l==r){CHECK(!result);continue;}int best=m,index=-100;
        for(int i=l;i<r;++i){int v=(a*i+b)%m;if(v<0)v+=m;if(v<best){best=v;index=i;}}CHECK(result&&result->value==best&&result->index==index);
        if(test<250){unsigned p=rng()%3,q=rng()%3;long long expected=0;for(int i=l;i<r;++i){int rem=(a*i+b)%m;if(rem<0)rem+=m;expected+=power(i,p)*power(rem,q);}CHECK(cp::polynomial_mod_sum(l,r,a,b,m,p,q)==expected);CHECK(cp::polynomial_mod_sum_mod(l,r,a,b,m,p,q,12)==static_cast<std::uint64_t>((expected%12+12)%12));}
    }
    const BigInt huge("1000000000000000000000000000000");
    auto moments=cp::floor_sum_moments(huge,7,21,-8,3);
    const BigInt s1=huge*(huge-1)/2,s2=huge*(huge-1)*(2*huge-1)/6,s3=s1*s1;
    CHECK(moments[1][2]==9*s3-12*s2+4*s1);CHECK(moments[0][1]==3*s1-2*huge);
    CHECK(cp::min_mod_linear(-huge,huge,huge+1,3,huge)->value==0);
    CHECK(cp::min_mod_linear(0,huge,0,-1,huge)->index==0);
    CHECK(cp::polynomial_mod_sum(0,huge,1,0,10,0,2)==huge/10*285);
    CHECK(cp::polynomial_mod_sum(0,huge,-1,0,10,0,1)==huge/10*45);
    CHECK(cp::floor_word_product(huge,17,31,9,BigInt(1),BigInt(0),BigInt(0),[](BigInt a,BigInt b){return a+b;})==(31*(huge-1)+9)/17);
    throws<std::invalid_argument>([]{cp::min_mod_linear(1,0,1,1,2);});
    throws<std::invalid_argument>([]{cp::floor_sum_big(1,0,1,1);});
    throws<std::invalid_argument>([]{cp::floor_sum_moments(-1,1,0,0,1);});
    throws<std::length_error>([]{cp::floor_sum_moments(10,2,1,0,13);});
    throws<std::length_error>([]{cp::polynomial_mod_sum(0,2,1,1,2,UINT_MAX,1);});
    throws<std::invalid_argument>([]{cp::polynomial_mod_sum_mod(0,2,1,1,2,1,1,0);});
    std::cout<<"floor: noncommutative words, signed moments, earliest minima, modular sums and huge inputs passed\n";
}
