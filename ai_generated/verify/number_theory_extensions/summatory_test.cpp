#include "common.hpp"
#include "../../number_theory_convolution/dirichlet_prefix_product.hpp"
#include "../../number_theory_convolution/dirichlet_inverse.hpp"
#include "../../number_theory_convolution/min25_sieve.hpp"
#include "../../number_theory_convolution/dirichlet_convolution.hpp"
#include "../../basic_utility/static_modint.hpp"
using cp::BigInt;using U=std::uint64_t;
std::optional<BigInt> unit(BigInt x){if(x==1||x==-1)return x;return std::nullopt;}
int phi(int n){int r=0;for(int i=1;i<=n;++i)r+=std::gcd(i,n)==1;return r;}
int main(){
    std::mt19937 rng(314);
    for(int n=0;n<=180;++n){std::vector<BigInt> a(n+1),b(n+1),pa(n+1),pb(n+1);
        for(int i=1;i<=n;++i){a[i]=int(rng()%7)-3;b[i]=int(rng()%7)-3;if(i==1)a[i]=n%2?-1:1;pa[i]=pa[i-1]+a[i];pb[i]=pb[i-1]+b[i];}
        cp::QuotientPrefix<BigInt> f(n,[&](U x){return pa[x];}),g(n,[&](U x){return pb[x];});
        auto product=cp::dirichlet_prefix_product(f,g);std::vector<BigInt> convolution(n+1),prefix(n+1);
        for(int i=1;i<=n;++i)for(int j=1;i*j<=n;++j)convolution[i*j]+=a[i]*b[j];
        for(int i=1;i<=n;++i)prefix[i]=prefix[i-1]+convolution[i];
        for(auto x:f.quotients()){CHECK(product.sum(x)==prefix[x]);CHECK(f.sum(x)==pa[x]);}CHECK(f.sum(0)==0);
        auto inverse=cp::dirichlet_inverse(a,unit);auto table_inverse=cp::dirichlet_prefix_inverse(f,unit);CHECK(inverse&&table_inverse);
        auto identity=cp::dirichlet_convolution(a,*inverse);std::vector<BigInt> pi(n+1);
        for(int i=1;i<=n;++i){CHECK(identity[i]==(i==1?1:0));pi[i]=pi[i-1]+(*inverse)[i];}for(auto x:f.quotients())CHECK(table_inverse->sum(x)==pi[x]);
        cp::QuotientPrefix<BigInt> ones(n,[](U x){return BigInt(x);});auto square=cp::dirichlet_prefix_product(ones,ones);BigInt divisor_sum=0;for(int i=1;i<=n;++i)divisor_sum+=n/i;CHECK(square.total()==divisor_sum);
        const auto prime_sums=cp::prime_sum_table(n);BigInt primes=0;for(int i=2;i<=n;++i){bool prime=true;for(int j=2;j<i;++j)if(i%j==0)prime=false;if(prime)primes+=i;}CHECK(prime_sums.total()==primes);
        auto totient=cp::min25_sieve<BigInt>(n,{-1,1},[](U p,unsigned,U power){return BigInt(power/p*(p-1));});BigInt total=0;for(int i=1;i<=n;++i)total+=phi(i);CHECK(totient.total()==total);
    }
    using M=cp::StaticModInt<12>;
    for(int n:{1,2,7,17,100,1000}){
        auto squares=cp::min25_sieve<M>(n,{0,0,1},[](U,unsigned,U power){return M(power)*M(power);});
        for(U x:squares.quotients())CHECK(squares.sum(x)==M((BigInt(x)*(x+1)*(2*x+1)/6%12).to_integral<int>()));
        auto divisor_count=cp::min25_sieve<BigInt>(n,{2},[](U,unsigned e,U){return BigInt(e+1);});
        auto dsum=cp::min25_sieve<BigInt>(n,{1,1},[](U p,unsigned e,U){BigInt r=1,power=1;for(unsigned j=0;j<e;++j){power*=p;r+=power;}return r;});
        for(auto x:dsum.quotients()){BigInt expected=0,count=0;for(U i=1;i<=x;++i){expected+=BigInt(i)*(x/i);count+=x/i;}CHECK(dsum.sum(x)==expected);CHECK(divisor_count.sum(x)==count);}
        // f(p) depends on p mod 4, supplied through an independent prime-prefix oracle.
        std::vector<int> spf(n+1),vals(n+1,1),prefix(n+1);
        for(int i=2;i<=n;++i){for(int p=2;p<=i;++p)if(i%p==0){spf[i]=p;break;}const int w=spf[i]%4==1?2:-1;vals[i]=w*vals[i/spf[i]];prefix[i]=prefix[i-1]+(spf[i]==i?w:0);}
        cp::QuotientPrefix<BigInt> prime_prefix(n,[&](U x){return BigInt(prefix[x]);});
        auto custom=cp::multiplicative_summatory_table(prime_prefix,[](U p,unsigned e,U){BigInt v=1;while(e--)v*=p%4==1?2:-1;return v;});
        BigInt expected=0;for(int i=1;i<=n;++i)expected+=vals[i];CHECK(custom.total()==expected);
    }
    auto large=cp::min25_sieve<BigInt>(100000000,{1},[](U,unsigned,U){return BigInt(1);});CHECK(large.total()==100000000);
    CHECK(cp::prime_sum(1000000)==37550402023LL);
    auto primes=cp::prime_sum_table(100000000);CHECK(primes.total()==BigInt("279209790387276"));
    cp::QuotientPrefix<BigInt> unit_table(100000000,[](U x){return BigInt(x);});
    auto inverse=cp::dirichlet_prefix_inverse(unit_table,unit);CHECK(inverse);CHECK(inverse->total()==1928);
    throws<std::length_error>([]{cp::QuotientPrefix<int>(100,[](U){return 0;},9);});
    throws<std::invalid_argument>([]{cp::QuotientPrefix<int>(10,[](U){return 0;}).sum(4);});
    throws<std::invalid_argument>([]{cp::dirichlet_prefix_product(cp::QuotientPrefix<int>(1,[](U){return 1;}),cp::QuotientPrefix<int>(2,[](U){return 1;}));});
    CHECK(!cp::dirichlet_inverse(std::vector<BigInt>{0,2},unit));
    CHECK(!cp::dirichlet_prefix_inverse(cp::QuotientPrefix<BigInt>(10,[](U x){return BigInt(2*x);}),unit));
    throws<std::out_of_range>([]{cp::polynomial_prime_sum_table<int>(100000,{0,0,1});});
    throws<std::length_error>([]{cp::polynomial_prime_sum_table<int>(100,{1,1,1},100,1);});
    std::cout<<"summatory: independent convolution/inverse, prime sums, Min25, composite rings and prime-oracle functions passed\n";
}
