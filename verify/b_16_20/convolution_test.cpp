#include "test_common.hpp"
#include "../../convolution/all.hpp"
#include <climits>
#include <random>
int main(){
 std::mt19937 rng(16182026);
 for(int n=0;n<=180;++n){V a(n),b(n);cp::RelaxedConvolution<> online(n);for(int i=0;i<n;++i){a[i]=int(rng()%200)-100;b[i]=int(rng()%200)-100;int expected=0;for(int j=0;j<=i;++j)expected=norm(expected+1LL*a[j]*b[i-j]);CHECK(online.append(a[i],b[i])==expected);}rejects<std::length_error>([&]{online.append(0,0);});}
 for(int trial=0;trial<250;++trial){int n=1+rng()%180,m=1+rng()%100;V a(n),b(m);for(auto&x:a)x=int(rng());for(auto&x:b)x=int(rng());CHECK(cp::convolution_large(a,b,8)==mul(a,b));CHECK((cp::convolution_large<17,3>(a,b,4)==mul(a,b,17)));if(n>=m){V expected(n-m+1);for(int k=0;k<=n-m;++k)for(int j=0;j<m;++j)expected[k]=norm(expected[k]+1LL*norm(a[k+j])*norm(b[j]));CHECK(cp::middle_product(a,b)==expected);}
  int start=int(rng()),ratio=int(rng());if(trial%7==0)ratio=0;auto values=cp::chirp_z(a,start,ratio,std::size_t(m));int point=norm(start);for(int k=0;k<m;++k){int expected=0;for(auto it=a.rbegin();it!=a.rend();++it)expected=norm(1LL*expected*point+*it);CHECK(values[k]==expected);point=norm(1LL*point*norm(ratio));}
 }
 for(int trial=0;trial<100;++trial){std::vector<std::size_t> sa{1+rng()%3,1+rng()%4,1+rng()%3},sb{1+rng()%3,1+rng()%3,1+rng()%2},out(3);std::size_t na=1,nb=1,no=1;for(int i=0;i<3;++i){na*=sa[i];nb*=sb[i];out[i]=sa[i]+sb[i]-1;no*=out[i];}V a(na),b(nb),expected(no);for(auto&x:a)x=int(rng()%100)-50;for(auto&x:b)x=int(rng()%100)-50;for(std::size_t i=0;i<na;++i)for(std::size_t j=0;j<nb;++j){auto ia=i,ib=j;std::size_t index=0,stride=1;for(int d=2;d>=0;--d){index+=(ia%sa[d]+ib%sb[d])*stride;stride*=out[d];ia/=sa[d];ib/=sb[d];}expected[index]=norm(expected[index]+1LL*a[i]*b[j]);}auto c=cp::multidimensional_convolution(a,sa,b,sb);CHECK(c.shape==out&&c.coefficients==expected);}
 CHECK(cp::multidimensional_convolution({-3},{},{4},{}).coefficients==V{998244341});
 V a{0,1,2,3,4},b{0,4,3,2,1};const auto gcd=cp::gcd_convolution(a,b),lcm=cp::lcm_convolution(a,b);V g(5),l(5);for(int i=1;i<5;++i)for(int j=1;j<5;++j){int d=std::gcd(i,j);g[d]+=a[i]*b[j];if(i/d*j<5)l[i/d*j]+=a[i]*b[j];}CHECK(gcd==g&&lcm==l);
 CHECK(cp::min_plus_convolution_convex_convex(V{0,1,4},V{0,2,7})==V({0,1,3,6,11}));
 // modulus17のNTT容量16より長い結果を明示的に検査。
 CHECK((cp::convolution_large<17,3>(V(43,1),V(47,1),8)==mul(V(43,1),V(47,1),17)));
 cp::RelaxedConvolution<17,3> small(8);for(int i=0;i<8;++i)CHECK(small.append(1,1)==i+1);
 cp::RelaxedConvolution<> large(32768);for(int i=0;i<32768;++i)CHECK(large.append(1,1)==i+1);
 rejects<std::invalid_argument>([]{cp::convolution_large(V{1},V{1},3);});rejects<std::invalid_argument>([]{cp::middle_product(V{},V{});});
 rejects<std::invalid_argument>([]{cp::multidimensional_convolution(V{1},{0},V{1},{1});});
 rejects<std::invalid_argument>([]{cp::multidimensional_convolution(V{1},{2},V{1},{1});});
 rejects<std::length_error>([]{cp::chirp_z<17,3>(V(8,1),1,2,4);});
 CHECK(cp::chirp_z({},1,2,3)==V(3));CHECK(cp::convolution_large({},V{1}).empty());
 std::cout<<"B convolution: streaming, blocked NTT, tensors, correlations, geometric evaluation passed\n";
}
