#include "test_common.hpp"
#include "../../fps/all.hpp"
#include "../../linear_recurrence/all.hpp"
#include <climits>
#include <random>
int main(){
 std::mt19937 rng(17182026);
 for(int trial=0;trial<180;++trial){std::size_t n=1+rng()%45,d=1+rng()%55;V a(d),b(n);for(auto&x:a)x=int(rng()%100)-50;for(auto&x:b)x=int(rng()%100)-50;CHECK(cp::fps_compose(a,b,n)==compose(a,b,n));b[0]=0;if(n==1)b.push_back(1);b[1]=1+rng()%50;const auto rev=cp::fps_compositional_inverse(b,n);V identity(n);if(n>1)identity[1]=1;CHECK(compose(b,rev,n)==identity);CHECK(compose(rev,b,n)==identity);
  V sparse(n);cp::SparseFPS terms;for(std::size_t j=0;j<n;++j)if(rng()%5==0){sparse[j]=int(rng()%7);terms.emplace_back(j,sparse[j]+3);terms.emplace_back(j,-3);}sparse[0]=1;terms.emplace_back(0,1); // 既存定数を相殺して1へ
  int c0=0;for(auto[k,v]:terms)if(k==0)c0+=v;terms.emplace_back(0,1-c0);
  const auto inv=cp::sparse_fps_inv(terms,n);CHECK(inv==inverse(sparse,n));CHECK(cp::sparse_fps_log(terms,n)==cp::fps_log(sparse,n));
  terms.emplace_back(0,-1);sparse[0]=0;CHECK(cp::sparse_fps_exp(terms,n)==exp_series(sparse,n));
  V expected(n);expected[0]=1;int exponent=rng()%7;for(int j=0;j<exponent;++j)expected=prefix(mul(expected,sparse),n);CHECK(cp::sparse_fps_pow(terms,exponent,n)==expected);
  V dense(n);for(auto&x:dense)x=int(rng()%100);dense[0]=1;cp::OnlineFPSInverse<> oi(n);const auto inverse_dense=inverse(dense,n);for(std::size_t j=0;j<n;++j)CHECK(oi.append(dense[j])==inverse_dense[j]);dense[0]=0;cp::OnlineFPSExp<> oe(n);const auto exp_dense=exp_series(dense,n);for(std::size_t j=0;j<n;++j)CHECK(oe.append(dense[j])==exp_dense[j]);
 }
 for(int trial=0;trial<100;++trial){std::size_t n=1+rng()%18;V points(n),a(n);for(std::size_t i=0;i<n;++i){points[i]=int(i)-7;a[i]=rng()%100;}cp::SubproductTree<> tree(points);CHECK(tree.product().size()==n+1);auto values=tree.evaluate(a);for(std::size_t i=0;i<n;++i)CHECK(values[i]==cp::polynomial_evaluate(a,points[i]));CHECK(tree.interpolate(values)==a);CHECK(tree.evaluate(V{1})==V(n,1));}
 cp::SubproductTree<> empty({});CHECK(empty.product()==V{1}&&empty.interpolate({}).empty());
 cp::SubproductTree<> duplicate({1,1});CHECK(duplicate.evaluate({0,1})==V({1,1}));rejects<std::domain_error>([&]{duplicate.interpolate({1,1});});
 for(int trial=0;trial<80;++trial){V a{int(rng()%10),int(rng()%10),1},b{int(rng()%10),1},c{int(rng()%10),1};auto g=cp::polynomial_gcd(mul(a,c),mul(b,c));CHECK(cp::polynomial_remainder(mul(a,c),g).empty());CHECK(cp::polynomial_remainder(mul(b,c),g).empty());CHECK(g.back()==1);
  std::uint64_t exponent=rng()%15;V expected{1};for(std::uint64_t i=0;i<exponent;++i)expected=cp::polynomial_remainder(mul(expected,a),b);CHECK(cp::polynomial_mod_pow(a,exponent,b)==expected);
 }
 CHECK(cp::polynomial_mod_pow({0,1},UINT64_MAX,{998244352,1})==V{1});CHECK(cp::polynomial_gcd({},{}).empty());
 // 重根と多項式部を含む部分分数を共通分母へ戻す。
 for(int trial=0;trial<60;++trial){V roots{-1,2,5};std::vector<std::size_t> multiplicities{1,2,3};V q{1};std::vector<V> factors;for(std::size_t i=0;i<roots.size();++i){V factor{1};for(std::size_t j=0;j<multiplicities[i];++j)factor=mul(factor,V{norm(-roots[i]),1});factors.push_back(factor);q=mul(q,factor);}V p(12);for(auto&x:p)x=int(rng()%101)-50;auto parts=cp::partial_fraction_decomposition(p,roots,multiplicities);V restored=mul(parts.polynomial_part,q);restored.resize(p.size());for(std::size_t i=0;i<roots.size();++i){V divisor{1};for(std::size_t j=1;j<=multiplicities[i];++j){divisor=mul(divisor,V{norm(-roots[i]),1});auto term=cp::polynomial_quotient(q,divisor);for(std::size_t k=0;k<term.size();++k)restored[k]=norm(restored[k]+1LL*term[k]*parts.coefficients[i][j-1]);}}for(auto&x:p)x=norm(x);CHECK(restored==p);}
 // factorial, Catalan型の多項式係数漸化式、消える先頭係数。
 auto factorial=cp::p_recursive_prefix(V{1},std::vector<V>{{-1,-1},{1}},80);int fact=1;for(int i=0;i<80;++i){if(i)fact=norm(1LL*fact*i);CHECK(factorial[i]==fact);}
 rejects<std::domain_error>([]{cp::p_recursive_prefix<5>(V{1},std::vector<V>{{1},{1,1}},7);});
 CHECK(cp::linear_recurrence_from_prefix(V{0,1,1,2,3,5,8,13},20)==6765);
 for(int trial=0;trial<80;++trial){std::size_t n=1+rng()%25;V a(n),b(n);for(auto&x:a)x=rng()%100;for(auto&x:b)x=rng()%100;V expected(n);std::vector<V> binomial(n,V(n));binomial[0][0]=1;for(std::size_t i=1;i<n;++i){binomial[i][0]=1;for(std::size_t j=1;j<=i;++j)binomial[i][j]=norm(binomial[i-1][j-1]+binomial[i-1][j]);}for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<=i;++j)expected[i]=norm(expected[i]+1LL*binomial[i][j]*a[j]%998244353*b[i-j]);CHECK(cp::egf_multiply(a,b,n)==expected);CHECK(cp::egf_from_ordinary(cp::egf_to_ordinary(a))==a);a[0]=0;CHECK(cp::egf_log(cp::egf_exp(a,n),n)==a);b[0]=1;auto iv=cp::egf_inv(b,n);V unit(n);unit[0]=1;CHECK(cp::egf_multiply(b,iv,n)==unit);
  V p{int(rng()%30),int(rng()%30),int(rng()%30)},q{1,2,3};auto coeff=prefix(mul(p,inverse(q,n)),n);CHECK(cp::generating_function_coefficients(p,q,n)==coeff);cp::RationalGeneratingFunction<> rational(p,q);CHECK(rational.prefix(n)==coeff);for(std::size_t i=0;i<n;++i)CHECK(rational.coefficient(i)==coeff[i]);CHECK((rational+rational).prefix(n)==cp::polynomial_add(coeff,coeff));CHECK((rational*rational).prefix(n)==prefix(mul(coeff,coeff),n));CHECK(cp::product_coefficient(std::vector<V>{p,q},n-1)==prefix(mul(p,q),n)[n-1]);
  int ratio=int(rng()%8);std::uint64_t count=rng()%120;int sum=0,weight=1;for(std::uint64_t k=0;k<count;++k){int value=0;for(auto it=p.rbegin();it!=p.rend();++it)value=norm(1LL*value*int(k)+*it);sum=norm(sum+1LL*value*weight);weight=norm(1LL*weight*ratio);}CHECK(cp::polynomial_geometric_sum(p,ratio,count)==sum);
 }
 CHECK(cp::polynomial_geometric_sum(V{1},1,UINT64_MAX)==int(UINT64_MAX%998244353));
 CHECK(cp::polynomial_geometric_sum(V{3,1},0,UINT64_MAX)==3);
 CHECK((cp::polynomial_geometric_sum<17,3>(V{1},1,12345)==12345%17));
 CHECK((cp::sparse_fps_exp<5>({},5)==V({1,0,0,0,0})));
 rejects<std::domain_error>([]{cp::sparse_fps_exp<5>({},6);});rejects<std::domain_error>([]{cp::fps_compositional_inverse(V{1,1},4);});
 rejects<std::domain_error>([]{cp::sparse_fps_inv({},4);});rejects<std::invalid_argument>([]{cp::partial_fraction_decomposition(V{1},V{2,2},std::vector<std::size_t>{1,1});});
 // 複数項・多項式係数の既知数列。
 auto squares=cp::p_recursive_prefix(V{1,4},std::vector<V>{{-9,-6,-1},{},{1,2,1}},40);
 for(int t=0;t<40;++t)CHECK(squares[t]==(t+1)*(t+1));
 for(int trial=0;trial<80;++trial){int root1=int(rng()%30),root2=root1+1;V common{int(rng()%20),int(rng()%20),1};CHECK(cp::polynomial_gcd(mul(common,V{-root1,1}),mul(common,V{-root2,1}))==common);V a{int(rng()%25),int(rng()%25),1};auto value=cp::polynomial_mod_pow(a,23,V{-root1,1});CHECK(value==cp::polynomial_trim(V{power(cp::polynomial_evaluate(a,root1),23)}));}
 cp::OnlineFPSInverse<> large_inverse(4096);cp::OnlineFPSExp<> large_exp(4096);
 for(int t=0;t<4096;++t){CHECK(large_inverse.append(t==0?1:t==1?-1:0)==1);CHECK(large_exp.append(t?power(t,998244351):0)==1);}
 V catalan(256);catalan[0]=1;for(int t=1;t<256;++t)for(int j=0;j<t;++j)catalan[t]=norm(catalan[t]+1LL*catalan[j]*catalan[t-1-j]);
 const auto reverted=cp::fps_compositional_inverse(V{0,1,1},256);for(int t=1;t<256;++t)CHECK(reverted[t]==(t%2?catalan[t-1]:norm(-catalan[t-1])));
 CHECK((cp::sparse_fps_inv<3>(cp::SparseFPS{{0,1},{1,-1}},80)==V(80,1)));
 CHECK((cp::sparse_fps_pow<5>(cp::SparseFPS{{2,1}},2,5)==V({0,0,0,0,1})));
 rejects<std::length_error>([]{cp::egf_inv<17,3>(V{1},9);});
 rejects<std::length_error>([]{cp::egf_exp<17,3>(V{0},9);});
 std::cout<<"B FPS/recurrence: composition, sparse/online operations, trees, fractions, EGF and rational coefficients passed\n";
}
