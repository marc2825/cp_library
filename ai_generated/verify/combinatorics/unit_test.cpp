#include <algorithm>
#include <array>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>
#include "combinatorics/all.hpp"
#define CHECK(...) do { if (!(__VA_ARGS__)) { std::cerr << "check failed at " << __LINE__ << ": " << #__VA_ARGS__ << '\n'; std::abort(); } } while (false)
template <class E, class F> void throws(F f) { bool caught = false; try { f(); } catch (const E&) { caught = true; } CHECK(caught); }
int bits(std::uint64_t mask) { int count = 0; for (; mask; mask &= mask - 1) ++count; return count; }
long long choose(int n, int k) { if (k<0||k>n) return 0; long long value=1;for(int i=1;i<=k;++i)value=value*(n-i+1)/i;return value; }
long long weak_compositions(int types, int total) {
    if (!types) return total == 0;
    long long count = 0; for (int take=0;take<=total;++take) count+=weak_compositions(types-1,total-take);return count;
}
void table_tests() {
    int valid=0,rejected=0;
    for(int modulus=1;modulus<=50;++modulus) {
        cp::PascalBinomial pascal(40,modulus);
        for(int n=0;n<=40;++n)for(int k=-1;k<=n+1;++k)CHECK(cp::binomial(pascal,n,k)==choose(n,k)%modulus);
        for(int bound=0;bound<=24;++bound) {
            bool unit=true;for(int i=1;i<=bound;++i)if(std::gcd(i,modulus)!=1)unit=false;
            const auto fact=cp::factorials(bound,modulus);long long value=1%modulus;
            for(int i=0;i<=bound;++i){if(i)value=value*i%modulus;CHECK(fact[i]==value);}
            if(!unit){throws<std::domain_error>([&]{cp::FactorialTable table(bound,modulus);});++rejected;continue;}
            ++valid;const cp::FactorialTable table(bound,modulus);
            CHECK(table.max_n()==bound && table.modulus()==modulus && table.factorials()==fact);
            for(int n=0;n<=bound;++n){CHECK(table.factorial(n)*table.inverse_factorial(n)%modulus==1%modulus);CHECK(table.inverse_factorials()[n]==table.inverse_factorial(n));for(int k=-1;k<=n+1;++k){CHECK(cp::binomial(table,n,k)==choose(n,k)%modulus);CHECK(cp::combinations(table,n,k)==cp::binomial(pascal,n,k));}}
        }
    }
    cp::FactorialTable table(50,1000000007);cp::PascalBinomial pascal(50,12);
    for(int n=0;n<=12;++n){std::vector<int> counts(n+1);for(int mask=0;mask<(1<<n);++mask)++counts[bits(mask)];for(int k=0;k<=n;++k)CHECK(cp::binomial(table,n,k)==counts[k]);}
    for(int n=0;n<=8;++n){std::array<long long,9> counts{};std::function<void(int,int)> visit=[&](int mask,int depth){++counts[depth];for(int i=0;i<n;++i)if(!(mask>>i&1))visit(mask|(1<<i),depth+1);};visit(0,0);for(int k=0;k<=n;++k)CHECK(cp::permutations(table,n,k)==counts[k]);}
    for(int n=0;n<=6;++n)for(int k=0;k<=9;++k){const auto count=weak_compositions(n,k);CHECK(cp::multiset_combination(table,n,k)==count);CHECK(cp::multiset_combination(pascal,n,k)==count%12);}
    for(int n=0;n<=10;++n){long long count=0;for(int mask=0;mask<(1<<(2*n));++mask){int depth=0;bool valid_path=true;for(int i=0;i<2*n;++i){depth+=(mask>>i&1)?1:-1;if(depth<0){valid_path=false;break;}}count+=valid_path&&depth==0;}CHECK(cp::catalan(table,n)==count && cp::catalan(pascal,n)==count%12);}
    // Catalanを独立した最初の括弧の分解DPと比較。合成数法・n>=法も含む。
    for(int modulus=1;modulus<=20;++modulus){cp::PascalBinomial p(60,modulus);std::vector<long long> catalans(31,0);catalans[0]=1%modulus;for(int n=1;n<=30;++n)for(int left=0;left<n;++left)catalans[n]=(catalans[n]+catalans[left]*catalans[n-1-left])%modulus;for(int n=0;n<=30;++n)CHECK(cp::catalan(p,n)==catalans[n]);}
    std::cout<<valid<<" valid factorial tables / "<<rejected<<" rejected nonunits; Pascal, subsets, ordered selections, weak compositions, bracket paths passed\n";
}
struct Mod12 {
    int value;
    Mod12(long long n=0):value(static_cast<int>((n%12+12)%12)){}
    Mod12& operator+=(const Mod12& x){value=(value+x.value)%12;return *this;}
    Mod12& operator-=(const Mod12& x){value=(value-x.value+12)%12;return *this;}
};
void inclusion_tests() {
    long long families=0;
    for(int n=0,ways=1;n<=4;++n,ways*=16)for(int code=0;code<ways;++code){int digits=code,united=0;std::array<int,4> sets{};for(int i=0;i<n;++i){sets[i]=digits%16;digits/=16;united|=sets[i];}
        std::vector<std::uint64_t> seen;
        const auto intersection=[&](std::uint64_t mask){seen.push_back(mask);int common=15;for(int i=0;i<n;++i)if(mask>>i&1)common&=sets[i];return static_cast<long long>(bits(common));};
        CHECK(cp::inclusion_exclusion_union<long long>(n,intersection)==bits(united));CHECK(seen.size()==(1ULL<<n)-1);for(std::size_t i=0;i<seen.size();++i)CHECK(seen[i]==i+1);
        seen.clear();CHECK(cp::inclusion_exclusion_complement<long long>(n,intersection)==4-bits(united));CHECK(seen.size()==(1ULL<<n));for(std::size_t i=0;i<seen.size();++i)CHECK(seen[i]==i);
        CHECK(cp::inclusion_exclusion_union<Mod12>(n,intersection).value==bits(united));CHECK(cp::inclusion_exclusion_complement<Mod12>(n,intersection).value==4-bits(united));++families;
    }
    CHECK(cp::inclusion_exclusion_union<long long>(0,[](std::uint64_t)->long long{throw std::runtime_error("must not call");})==0);
    CHECK(cp::inclusion_exclusion_complement<long long>(0,[](std::uint64_t mask){CHECK(mask==0);return 7LL;})==7);
    for(int n:{-1,64,INT_MAX}){throws<std::invalid_argument>([&]{cp::inclusion_exclusion_union<long long>(n,[](std::uint64_t){return 0LL;});});throws<std::invalid_argument>([&]{cp::inclusion_exclusion_complement<long long>(n,[](std::uint64_t){return 0LL;});});}
    struct Stop{};throws<Stop>([]{cp::inclusion_exclusion_union<long long>(63,[](std::uint64_t mask)->long long{CHECK(mask==1);throw Stop{};});});throws<Stop>([]{cp::inclusion_exclusion_complement<long long>(63,[](std::uint64_t mask)->long long{CHECK(mask==0);throw Stop{};});});
    for(std::uint64_t mask:{0ULL,1ULL,1ULL<<63,ULLONG_MAX,ULLONG_MAX>>1})CHECK(cp::combinatorics_detail::odd_popcount(mask)==static_cast<bool>(bits(mask)%2));
    throws<std::overflow_error>([]{cp::inclusion_exclusion_union<long long>(2,[](std::uint64_t){return LLONG_MAX;});});
    throws<std::overflow_error>([]{cp::inclusion_exclusion_complement<unsigned long long>(2,[](std::uint64_t){return 1ULL;});});
    long long callbacks=0;CHECK(cp::inclusion_exclusion_union<long long>(22,[&](std::uint64_t){++callbacks;return 1LL;})==1);CHECK(callbacks==(1LL<<22)-1);callbacks=0;CHECK(cp::inclusion_exclusion_complement<long long>(22,[&](std::uint64_t){++callbacks;return 1LL;})==0);CHECK(callbacks==(1LL<<22));
    std::cout<<families<<" set families; union/complement/mod12, empty subset, 63-bit boundary and 22-set enumeration passed\n";
}
void bounds_and_large() {
    cp::FactorialTable zero(0),small(8),one(8,1);cp::PascalBinomial p(8,12);
    CHECK(cp::binomial(zero,0,0)==1 && zero.factorial(0)==1 && zero.inverse_factorial(0)==1);
    CHECK(cp::binomial(one,8,4)==0 && cp::catalan(one,4)==0 && cp::multiset_combination(one,0,0)==0);
    for(long long bad:{LLONG_MIN,-1LL}){CHECK(cp::binomial(small,bad,0)==0 && cp::permutations(small,bad,0)==0 && cp::catalan(small,bad)==0 && cp::multiset_combination(small,bad,0)==0);CHECK(cp::binomial(p,bad,0)==0);throws<std::out_of_range>([&]{small.factorial(bad);});throws<std::out_of_range>([&]{small.inverse_factorial(bad);});}
    CHECK(cp::binomial(small,LLONG_MAX,LLONG_MIN)==0 && cp::permutations(small,LLONG_MAX,-1)==0);
    CHECK(cp::multiset_combination(small,LLONG_MAX,0)==1 && cp::multiset_combination(small,0,LLONG_MAX)==0);
    throws<std::out_of_range>([&]{cp::multiset_combination(small,LLONG_MAX,LLONG_MAX);});throws<std::out_of_range>([&]{cp::catalan(small,LLONG_MAX);});throws<std::out_of_range>([&]{cp::binomial(small,LLONG_MAX,0);});throws<std::out_of_range>([&]{cp::binomial(p,LLONG_MAX,0);});throws<std::out_of_range>([&]{small.inverse_factorial(9);});throws<std::out_of_range>([&]{cp::permutations(small,9,0);});
    for(long long bad:{0LL,-1LL,LLONG_MIN}){throws<std::invalid_argument>([&]{cp::factorials(0,bad);});throws<std::invalid_argument>([&]{cp::FactorialTable t(0,bad);});throws<std::invalid_argument>([&]{cp::PascalBinomial t(0,bad);});}
    throws<std::invalid_argument>([]{cp::FactorialTable t(-1);});throws<std::invalid_argument>([]{cp::PascalBinomial t(-1);});
    for(long long modulus:{LLONG_MAX,LLONG_MAX-1,4294967291LL}){cp::PascalBinomial big(60,modulus);for(int n=0;n<=60;++n)for(int k=0;k<=n;++k)CHECK(cp::binomial(big,n,k)==choose(n,k)%modulus);}
    const int bound=2000000;const long long mod=1000000007;const cp::FactorialTable large(bound,mod);long long value=1;
    for(int n=0;n<=bound;++n){if(n)value=value*n%mod;CHECK(large.factorial(n)==value);CHECK(value*large.inverse_factorial(n)%mod==1);if(n)CHECK(cp::binomial(large,n,1)==n && cp::permutations(large,n,1)==n);}
    cp::PascalBinomial large_pascal(1500,12);for(int n=0;n<=1500;++n){long long sum=0;for(long long value:large_pascal.rows()[n])sum=(sum+value)%12;CHECK(sum==cp::mod_pow(2,n,12));}
    std::cout<<"invalid/trivial/overflow-safe bounds; 64-bit moduli; 2-million factorial entries and 1500-row composite Pascal table passed\n";
}
int main(){table_tests();inclusion_tests();bounds_and_large();}
