#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
#define CHECK(...) do { if (!(__VA_ARGS__)) { std::cerr << __FILE__ << ':' << __LINE__ << ": " << #__VA_ARGS__ << '\n'; std::abort(); } } while (false)
using V = std::vector<int>;
inline int norm(long long x, int p = 998244353) { x %= p; return int(x < 0 ? x + p : x); }
inline int power(int a, std::uint64_t e, int p = 998244353) { int r=1; a=norm(a,p); for(;e;e>>=1,a=norm(1LL*a*a,p)) if(e&1)r=norm(1LL*r*a,p);return r; }
inline V mul(const V& a, const V& b, int p = 998244353) {
 if(a.empty()||b.empty())return {};
 V c(a.size()+b.size()-1);
 for(std::size_t i=0;i<a.size();++i)for(std::size_t j=0;j<b.size();++j)c[i+j]=norm(c[i+j]+1LL*norm(a[i],p)*norm(b[j],p),p);
 return c;
}
inline V prefix(V a,std::size_t n){a.resize(n);return a;}
inline V compose(const V& a,const V& b,std::size_t n,int p=998244353){V c(n),v(n);if(n)v[0]=1;for(auto x:a){for(std::size_t j=0;j<n;++j)c[j]=norm(c[j]+1LL*x*v[j],p);v=prefix(mul(v,b,p),n);}return c;}
inline V inverse(const V& a,std::size_t n,int p=998244353){V b(n);if(!n)return b;b[0]=power(a[0],p-2,p);for(std::size_t i=1;i<n;++i){int sum=0;for(std::size_t j=1;j<=i&&j<a.size();++j)sum=norm(sum+1LL*a[j]*b[i-j],p);b[i]=norm(-1LL*sum*b[0],p);}return b;}
inline V exp_series(const V& a,std::size_t n,int p=998244353){V b(n);if(n)b[0]=1;for(std::size_t i=1;i<n;++i){int s=0;for(std::size_t j=1;j<=i&&j<a.size();++j)s=norm(s+1LL*j*a[j]%p*b[i-j],p);b[i]=norm(1LL*s*power(int(i),p-2,p),p);}return b;}
inline V subset_mul(const V& a,const V& b,int p=998244353){V c(a.size());for(std::size_t s=0;s<c.size();++s)for(auto t=s;;t=(t-1)&s){c[s]=norm(c[s]+1LL*norm(a[t],p)*norm(b[s^t],p),p);if(!t)break;}return c;}
template<class E=std::exception,class F>void rejects(F f){bool caught=false;try{f();}catch(const E&){caught=true;}CHECK(caught);}
