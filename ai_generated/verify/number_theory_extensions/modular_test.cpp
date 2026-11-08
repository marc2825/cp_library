#include "common.hpp"
#include "../../number_theory_basic/segmented_sieve.hpp"
#include "../../number_theory_basic/multiplicative_order.hpp"
#include "../../number_theory_basic/batch_inversion.hpp"
#include "../../number_theory_basic/discrete_log_table.hpp"
#include "../../number_theory_basic/mod_kth_root.hpp"
#include "../../combinatorics/prime_power_binomial.hpp"
#include "../../combinatorics/large_factorial.hpp"
#include "../../basic_utility/static_modint.hpp"
using U = std::uint64_t;
bool prime(U x) { if(x<2)return false;for(U d=2;d<=x/d;++d)if(x%d==0)return false;return true; }
int main() {
    for(U left=0;left<100;++left)for(U width=0;width<30;++width) {
        cp::SegmentedSieve sieve(left,left+width);std::vector<U> expected;
        for(U x=left;x<left+width;++x){if(prime(x))expected.push_back(x);if(x){U value=1;for(auto [p,e]:sieve.factorization(x)){CHECK(prime(p));for(int i=0;i<e;++i)value*=p;}CHECK(value==x);}}
        CHECK(sieve.primes()==expected);
    }
    cp::SegmentedSieve large(1000000000000ULL,1000000000250ULL);
    for(U x=1000000000000ULL;x<1000000000250ULL;++x)CHECK(large.factorization(x)==cp::prime_factorization_64(x));
    throws<std::domain_error>([]{cp::SegmentedSieve(0,2).factorization(0);});
    throws<std::out_of_range>([]{cp::SegmentedSieve(1,2).factorization(2);});
    throws<std::invalid_argument>([]{cp::SegmentedSieve(5,4);});
    throws<std::length_error>([]{cp::SegmentedSieve(0,10,9);});
    throws<std::length_error>([]{cp::SegmentedSieve(1000000,1000001,10,999);});
    for(U m=1;m<=120;++m){U lambda=1;
        for(U a=0;a<m;++a){auto order=cp::multiplicative_order(a,m);if(std::gcd(a,m)!=1){CHECK(!order);continue;}
            U x=1%m,k=0;do{x=x*a%m;++k;}while(x!=1%m);CHECK(order&&*order==k);lambda=std::lcm(lambda,k);
        }CHECK(cp::carmichael_lambda(m)==lambda);
        for(U a=0;a<m;++a){cp::DiscreteLogTable table(a,m);std::vector<std::optional<U>> expected(m);U x=1%m;
            for(U i=0;!expected[x];++i){expected[x]=i;x=x*a%m;}for(U b=0;b<m;++b)CHECK(table.query(b)==expected[b]);
        }
        for(U a=0;a<m;++a)for(U b=0;b<m;++b){auto inv=cp::batch_inverse_mod({a,b},m);CHECK(bool(inv)==(std::gcd(a,m)==1&&std::gcd(b,m)==1));if(inv){CHECK(a*(*inv)[0]%m==1%m);CHECK(b*(*inv)[1]%m==1%m);}}
    }
    const U top=std::numeric_limits<U>::max(), p64=18446744073709551557ULL;
    CHECK(cp::carmichael_lambda(U(1)<<63)==(U(1)<<61));
    CHECK(cp::multiplicative_order(top-1,top)==2);
    CHECK(cp::multiplicative_order(p64-1,p64)==2);
    auto inv=cp::batch_inverse_mod({p64-2,p64-1,2},p64);CHECK(inv);for(U i=0;i<3;++i)CHECK(cp::nt_detail::mul(std::vector<U>{p64-2,p64-1,2}[i],(*inv)[i],p64)==1);
    CHECK(cp::DiscreteLogTable(0,top).query(0)==1);
    CHECK(cp::DiscreteLogTable(2,U(1)<<63).query(0)==63);
    throws<std::length_error>([]{cp::DiscreteLogTable(3,1000000007,10);});
    throws<std::invalid_argument>([]{cp::DiscreteLogTable(0,0);});
    throws<std::invalid_argument>([]{cp::carmichael_lambda(0);});
    throws<std::invalid_argument>([]{cp::batch_inverse_mod({},0);});
    using M=cp::StaticModInt<12>;int inverse_calls=0;
    auto inverse=[&](M a){++inverse_calls;return a.try_inv();};
    auto units=cp::batch_inverse(std::vector<M>{1,5,7,11},inverse);CHECK(units&&inverse_calls==1);
    CHECK(!cp::batch_inverse(std::vector<M>{1,6},inverse));CHECK(inverse_calls==2);
    CHECK(cp::batch_inverse(std::vector<M>{},inverse)->empty());CHECK(inverse_calls==2);
    for(U m=1;m<=180;++m){cp::PrimePowerBinomial choose(m);std::vector<cp::BigInt> row{1};
        for(U n=0;n<=70;++n){for(U k=0;k<=n;++k)CHECK(choose.choose(n,k)==(row[k]%m).to_integral<U>());CHECK(choose.choose(n,n+1)==0);
            std::vector<cp::BigInt> next(n+2,1);for(U k=1;k<=n;++k)next[k]=row[k-1]+row[k];row=std::move(next);
        }
    }
    for(U m:{8,9,25,72,1000,65536}){cp::PrimePowerBinomial choose(m);cp::BigInt binomial=1;
        for(U k=0;k<50;++k){CHECK(choose.choose(top,k)==(binomial%m).to_integral<U>());CHECK(choose.choose(top,k)==choose.choose(top,top-k));binomial=binomial*(cp::BigInt(top)-k)/(k+1);}
    }
    throws<std::invalid_argument>([]{cp::PrimePowerBinomial(0);});
    throws<std::length_error>([]{cp::PrimePowerBinomial(1024,1024);});
    CHECK(cp::PrimePowerBinomial(1024,1025).choose(10,3)==120);
    for(int p=2;p<=150;++p)if(prime(p)){int f=1;for(int n=0;n<=p+1;++n){if(n)f=int(static_cast<long long>(f)*n%p);CHECK(cp::large_factorial_mod(n,p)==f);}}
    for(int p:{998244353,1000000007}){int f=1;for(int n=1;n<=1000000;++n)f=int(static_cast<long long>(f)*n%p);CHECK(cp::large_factorial_mod(1000000,p)==f);CHECK(cp::large_factorial_mod(p-1,p)==p-1);CHECK(cp::large_factorial_mod(p,p)==0);}
    // Exercise a genuinely sublinear query; use a second prime-field identity at the boundary.
    CHECK(cp::large_factorial_mod(500000003,1000000007)==cp::large_factorial_mod(500000002,1000000007)*1LL*500000003%1000000007);
    throws<std::invalid_argument>([]{cp::large_factorial_mod(10,15);});
    throws<std::length_error>([]{cp::large_factorial_mod(100,1009,9);});
    for(U p=2;p<70;++p)if(prime(p))for(U k=0;k<16;++k)for(U a=0;a<p;++a){std::vector<U> expected;
        for(U x=0;x<p;++x){U y=1;for(U i=0;i<k;++i)y=y*x%p;if(y==a)expected.push_back(x);}CHECK(cp::mod_kth_roots(a,k,p).enumerate()==expected);
    }
    CHECK(cp::mod_kth_roots(0,2,p64).enumerate()==std::vector<U>{0});
    throws<std::invalid_argument>([]{cp::mod_kth_roots(1,2,9);});
    throws<std::length_error>([]{cp::mod_kth_roots(1,0,101).enumerate(100);});
    std::cout<<"modular: sieve, orders, batch inverses, huge binomials/factorials, BSGS and roots passed\n";
}
