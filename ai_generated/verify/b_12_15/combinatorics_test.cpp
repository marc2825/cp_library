#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <vector>
#include "combinatorics/all.hpp"
#define CHECK(x) do{if(!(x)){std::cerr<<"line "<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(false)
constexpr long long mod=998244353;
std::mt19937 rng(315122026);int draw(int n){return int(rng()%n);}
long long choose(int n,int k){if(k<0 || k>n)return 0;k=std::min(k,n-k);long long x=1;for(int i=1;i<=k;++i)x=x*(n-i+1)/i;return x;}
std::vector<int> multiply(const std::vector<int>& a,const std::vector<int>& b,int n){std::vector<int> c(n);for(int i=0;i<static_cast<int>(a.size()) && i<n;++i)for(int j=0;j<static_cast<int>(b.size()) && i+j<n;++j)c[i+j]=int((c[i+j]+1LL*a[i]*b[j])%mod);return c;}
std::vector<int> compose(const std::vector<int>& a,const std::vector<int>& b,int n){std::vector<int> result(n),power(n);if(n)power[0]=1;for(int x:a){for(int i=0;i<n;++i)result[i]=int((result[i]+1LL*x*power[i])%mod);power=multiply(power,b,n);}return result;}
int main(){
 auto euler=cp::eulerian_numbers(9);for(int n=0;n<=8;++n){std::vector<int> p(n);std::iota(p.begin(),p.end(),0);std::vector<long long> counts(n+1);do{int ascents=0;for(int i=1;i<n;++i)ascents+=p[i-1]<p[i];++counts[ascents];}while(std::next_permutation(p.begin(),p.end()));CHECK(euler[n]==counts);}
 cp::FactorialTable table(80);for(int n=0;n<=8;++n){std::vector<long long> peaks(n+1);for(unsigned mask=0;mask<(1u<<(2*n));++mask){int height=0,k=0;bool good=true;for(int i=0;i<2*n;++i){bool up=mask>>i&1;height+=up?1:-1;if(height<0)good=false;if(i && !up && (mask>>(i-1)&1))++k;}if(good && height==0)++peaks[k];}for(int k=0;k<=n;++k)CHECK(cp::narayana(table,n,k)==peaks[k]);}
 auto bernoulli=cp::bernoulli_numbers(35);for(int k=0;k<=30;++k)for(int bound=0;bound<=18;++bound){long long sum=0;for(int x=0;x<bound;++x)sum=(sum+cp::mod_pow(x,k,mod))%mod;long long formula=0;for(int j=0;j<=k;++j)formula=(formula+cp::mod_multiply(cp::binomial(table,k+1,j),cp::mod_multiply(bernoulli[j],cp::mod_pow(bound,k+1-j,mod),mod),mod))%mod;formula=cp::mod_multiply(formula,*cp::mod_inverse(k+1,mod),mod);CHECK(formula==sum);}
 for(int trial=0;trial<160;++trial){int n=draw(20);std::vector<int> phi(1+draw(8)),f(1+draw(22));for(auto& x:phi)x=draw(8);for(auto& x:f)x=draw(8);std::vector<int> g(n+1);for(int iteration=0;iteration<=n;++iteration){auto composed=compose(phi,g,n+1);for(int i=n;i>0;--i)g[i]=composed[i-1];g[0]=0;}auto expected=compose(f,g,n+1);CHECK(cp::lagrange_inversion_coefficient(phi,f,n)==expected[n]);}
 for(int trial=0;trial<300;++trial){int n=draw(17);long long modulus=trial%3==0?12:trial%3==1?1:mod;std::vector<long long> a(n),normalized(n);for(int i=0;i<n;++i){a[i]=draw(61)-30;normalized[i]=(a[i]%modulus+modulus)%modulus;}auto b=cp::stirling_transform(a,modulus);CHECK(cp::stirling_transform(b,modulus,true)==normalized);auto binom=cp::binomial_transform(a,modulus);for(int i=0;i<n;++i){long long direct=0;for(int k=0;k<=i;++k)direct=(direct+cp::mod_multiply(choose(i,k),a[k],modulus)+modulus)%modulus;CHECK(binom[i]==direct);}CHECK(cp::binomial_transform(binom,modulus,true)==normalized);
  for(bool rising:{false,true}){auto c=cp::power_to_factorial_basis(a,modulus,rising);CHECK(cp::factorial_basis_to_power(c,modulus,rising)==normalized);for(int x=-3;x<=5;++x){long long powers=0,factorials=0,product=1%modulus;for(auto it=a.rbegin();it!=a.rend();++it)powers=(cp::mod_multiply(powers,x,modulus)+(*it%modulus+modulus)%modulus)%modulus;for(int k=0;k<n;++k){factorials=(factorials+cp::mod_multiply(c[k],product,modulus))%modulus;product=cp::mod_multiply(product,rising?x+k:x-k,modulus);}CHECK(powers==factorials);}}
 }
 for(int n=0;n<=8;++n){std::vector<long long> a(n+1);for(int k=0;k<=n;++k)a[k]=k*k+3;long long expected=0;std::function<void(int,int)> partitions=[&](int at,int blocks){if(at==n){expected+=a[blocks];return;}for(int k=0;k<=blocks;++k)partitions(at+1,std::max(blocks,k+1));};partitions(0,0);CHECK(cp::stirling_transform(a)[n]==expected%mod);}
 for(int trial=0;trial<200;++trial){int n=draw(35),count=draw(45);std::vector<int> coefficient(n),samples(n);for(auto& x:coefficient)x=draw(61)-30;auto eval=[&](long long x){long long value=0;for(auto it=coefficient.rbegin();it!=coefficient.rend();++it)value=(cp::mod_multiply(value,x,mod)+(*it%mod+mod)%mod)%mod;return int(value);};for(int i=0;i<n;++i)samples[i]=eval(i);long long first=trial%7==0?std::numeric_limits<long long>::min():draw(101)-50;auto shifted=cp::shift_sample_points(samples,first,count);for(int i=0;i<count;++i){long long x=(cp::detail::normalized_remainder(first,mod)+i)%mod;CHECK(shifted[i]==eval(x));}}
 {auto shifted=cp::shift_sample_points<17,3>({0,1,4,9},15,7);for(int i=0;i<7;++i)CHECK(shifted[i]==(15+i)%17*((15+i)%17)%17);}
 for(long long modulus:{1LL,12LL,mod}){cp::BinomialPrefixSum prefix(50,modulus);for(int n=0;n<=50;++n){long long sum=0;for(int k=-1;k<=n+2;++k){if(k>=0 && k<=n)sum=(sum+choose(n,k)%modulus)%modulus;CHECK(prefix(n,k)==sum);}for(int l=-2;l<n+2;++l)for(int r=l;r<n+3;++r){long long expect=0;for(int k=std::max(0,l);k<std::min(n+1,r);++k)expect=(expect+choose(n,k)%modulus)%modulus;CHECK(prefix.sum(n,l,r)==expect);}}CHECK(prefix.sum(10,std::numeric_limits<long long>::min(),std::numeric_limits<long long>::max())==1024%modulus);}
 {bool rejected=false;try{cp::bernoulli_numbers(4,5);}catch(const std::domain_error&){rejected=true;}CHECK(rejected);CHECK(cp::shift_sample_points(std::vector<int>{},5,3)==std::vector<int>({0,0,0}));CHECK(cp::lagrange_inversion_coefficient({1},{8},0)==8);}
 std::cout<<"combinatorics: permutations, Dyck paths, power sums, formal fixed points, set partitions and direct polynomial evaluation passed\n";
}
