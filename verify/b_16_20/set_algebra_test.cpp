#include "test_common.hpp"
#include "../../set_function/all.hpp"
#include "../../linear_algebra/all.hpp"
#include <climits>
#include <functional>
#include <numeric>
#include <random>
using M=cp::Matrix<int>;
V mv(const M&a,const V&v,int p){V r(a.rows());for(std::size_t i=0;i<a.rows();++i)for(std::size_t j=0;j<a.cols();++j)r[i]=norm(r[i]+1LL*a(i,j)*v[j],p);return r;}
M mm(const M&a,const M&b,int p){M c(a.rows(),b.cols());for(std::size_t i=0;i<a.rows();++i)for(std::size_t j=0;j<b.cols();++j)for(std::size_t k=0;k<a.cols();++k)c(i,j)=norm(c(i,j)+1LL*a(i,k)*b(k,j),p);return c;}
int determinant(const M&a,int p){const int n=int(a.rows());V perm(n);std::iota(perm.begin(),perm.end(),0);int result=0;do{int term=1,inversions=0;for(int i=0;i<n;++i){term=norm(1LL*term*a(i,perm[i]),p);for(int j=i+1;j<n;++j)inversions+=perm[i]>perm[j];}result=norm(result+(inversions%2?-term:term),p);}while(std::next_permutation(perm.begin(),perm.end()));return result;}
M evaluate_matrix(const V&poly,const M&a,int p){M result(a.rows(),a.cols());for(auto it=poly.rbegin();it!=poly.rend();++it){result=mm(result,a,p);for(std::size_t i=0;i<a.rows();++i)result(i,i)=norm(result(i,i)+*it,p);}return result;}
bool zero(const M&a){return std::all_of(a.data().begin(),a.data().end(),[](int x){return x==0;});}
template<int P>void sets(){std::mt19937 rng(P);for(int bits=0;bits<=6;++bits)for(int trial=0;trial<35;++trial){std::size_t n=std::size_t(1)<<bits;V a(n),b(n);for(auto&x:a)x=rng()%P;for(auto&x:b)x=rng()%P;CHECK(cp::subset_convolution_mod<P>(a,b)==subset_mul(a,b,P));a[0]=1;auto inv=cp::sps_inv<P>(a);V unit(n);unit[0]=1;CHECK(subset_mul(a,inv,P)==unit);a[0]=0;auto exponential=cp::sps_exp<P>(a);CHECK(cp::sps_log<P>(exponential)==a);
 V partition(n);partition[0]=1;
 // 独立したrestricted-growth stringで全てのラベルなし分割を列挙。
 for(std::size_t mask=1;mask<n;++mask){V elements;for(int i=0;i<bits;++i)if(mask&(std::size_t(1)<<i))elements.push_back(i);std::vector<std::size_t> blocks;
 std::function<void(std::size_t)> visit=[&](std::size_t at){if(at==elements.size()){int weight=1;for(auto block:blocks)weight=norm(1LL*weight*a[block],P);partition[mask]=norm(partition[mask]+weight,P);return;}for(std::size_t j=0;j<blocks.size();++j){blocks[j]|=std::size_t(1)<<elements[at];visit(at+1);blocks[j]^=std::size_t(1)<<elements[at];}blocks.push_back(std::size_t(1)<<elements[at]);visit(at+1);blocks.pop_back();};visit(0);}
 CHECK(exponential==partition);auto by=cp::set_partition_by_blocks<P>(a);for(std::size_t s=0;s<n;++s){int sum=0;for(const auto&row:by)sum=norm(sum+row[s],P);CHECK(sum==partition[s]);}CHECK(cp::set_partition_sum<P>(a)==partition);
 V outer{1,2,3,4,5,6,7,8,9},inner=b,result(n),term(n);term[0]=1;for(auto x:outer){for(std::size_t s=0;s<n;++s)result[s]=norm(result[s]+1LL*x*term[s],P);term=subset_mul(term,inner,P);}CHECK(cp::sps_compose<P>(outer,inner)==result);
 }}
template<int P>void algebra(){std::mt19937 rng(P*328u);for(int trial=0;trial<150;++trial){const std::size_t n=rng()%5;M a(n,n);std::vector<cp::SparseMatrix<int>::Entry> entries;for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j){a(i,j)=int(rng()%P);entries.push_back({i,j,a(i,j)+1});entries.push_back({i,j,-1});}cp::SparseMatrix<int> sparse(n,n,entries);auto apply=[&](const V&v){return cp::sparse_matrix_vector_multiply_mod<P>(sparse,v);};V v(n),u(n);for(auto&x:v)x=int(rng()%P);for(auto&x:u)x=int(rng()%P);CHECK(apply(v)==mv(a,v,P));CHECK(sparse.multiply(v)==mv(a,v,998244353));auto sequence=cp::black_box_krylov_sequence<P>(n,apply,u,v,2*n+3);auto state=v;for(std::size_t k=0;k<sequence.size();++k){int expected=0;for(std::size_t j=0;j<n;++j)expected=norm(expected+1LL*u[j]*state[j],P);CHECK(sequence[k]==expected);state=mv(a,state,P);}const auto projected=cp::wiedemann_polynomial<P>(n,apply,u,v);for(std::size_t i=0;i+projected.size()<=sequence.size();++i){int sum=0;for(std::size_t j=0;j<projected.size();++j)sum=norm(sum+1LL*projected[j]*sequence[i+j],P);CHECK(sum==0);}
 const auto minimum=cp::matrix_minimal_polynomial_mod<P>(a);CHECK(minimum.back()==1&&minimum.size()<=n+1);CHECK(zero(evaluate_matrix(minimum,a,P)));
 // 小体で、より低次数のmonic多項式を全探索して最小性を検証。
 if(P==3&&n<=4){for(std::size_t degree=0;degree+1<minimum.size();++degree){std::size_t count=1;for(std::size_t j=0;j<degree;++j)count*=P;for(std::size_t code=0;code<count;++code){auto x=code;V f(degree+1);f[degree]=1;for(std::size_t j=0;j<degree;++j){f[j]=x%P;x/=P;}CHECK(!zero(evaluate_matrix(f,a,P)));}}}
 if(n){auto direct=v;auto exponent=std::uint64_t(rng()%25);for(std::uint64_t j=0;j<exponent;++j)direct=mv(a,direct,P);CHECK(cp::black_box_power_apply<P>(n,apply,v,exponent,minimum)==direct);const auto solution=cp::wiedemann_solve<P>(n,apply,v,32,trial);if(solution)CHECK(mv(a,*solution,P)==v);if(determinant(a,P)){CHECK(solution.has_value());const auto known=cp::black_box_solve<P>(n,apply,v,minimum);CHECK(known&&mv(a,*known,P)==v);}}
 auto inverse=cp::matrix_inverse_mod<P>(a);if(!inverse)continue;
 std::size_t k=rng()%4;M update_u(n,k),update_v(n,k);for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<k;++j){update_u(i,j)=rng()%P;update_v(i,j)=rng()%P;}auto delta=mm(update_u,update_v.transposed(),P);M changed=a;for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)changed(i,j)=norm(changed(i,j)+delta(i,j),P);const auto updated=cp::low_rank_inverse_update_mod<P>(*inverse,update_u,update_v);CHECK(updated==cp::matrix_inverse_mod<P>(changed));int ratio=cp::low_rank_determinant_ratio_mod<P>(*inverse,update_u,update_v);CHECK(norm(1LL*determinant(a,P)*ratio,P)==determinant(changed,P));
 }}
int main(){sets<2>();sets<3>();sets<998244353>();algebra<17>();algebra<3>();
 // 非NTT素数でWiedemann、矩形CSR、重複、空次元、最上位uint64指数。
 cp::SparseMatrix<int> rect(2,3,{{0,1,2},{0,1,3},{1,2,-1}});CHECK(rect.multiply({7,8,9})==V({40,-9}));
 auto identity=[](const V&v){return v;};CHECK(cp::black_box_power_apply(2,identity,V{2,3},UINT64_MAX,V{998244352,1})==V({2,3}));
 auto answer=cp::wiedemann_solve<1000000007>(2,identity,V{2,3});CHECK(answer==std::optional<V>(V{2,3}));
 CHECK(cp::matrix_minimal_polynomial_mod(M(0,0))==V{1});CHECK(cp::matrix_minimal_polynomial_mod(M{{0,1},{0,0}})==V({0,0,1}));
 CHECK(cp::matrix_minimal_polynomial_mod(M{{2,0},{0,2}})==V({998244351,1}));
 cp::Graph<int> graph(3);graph.add_edge(0,1,2);graph.add_edge(1,2,3);graph.add_edge(0,2,5);CHECK(cp::count_weighted_spanning_trees(graph)==31);
 rejects<std::invalid_argument>([]{cp::sps_exp(V{});});rejects<std::domain_error>([]{cp::sps_log(V{2,0});});
 rejects<std::invalid_argument>([]{cp::black_box_krylov_sequence(2,[](const V&){return V(1);},V(2),V(2),3);});
 rejects<std::domain_error>([&]{cp::black_box_power_apply(2,identity,V{1,1},2,V{1,1});});
 rejects<std::out_of_range>([]{cp::SparseMatrix<int>(1,1,{{1,0,2}});});
 rejects<std::overflow_error>([]{cp::SparseMatrix<int>(1,1,{{0,0,INT_MAX}}).multiply(V{2});});
 rejects<std::invalid_argument>([]{cp::low_rank_inverse_update_mod(M(2,2),M(1,1),M(2,1));});
 cp::SparseMatrix<int> composite(1,1,{{0,0,INT_MIN},{0,0,INT_MAX}});CHECK((cp::sparse_matrix_vector_multiply_mod<12>(composite,V{INT_MAX})==V{norm(-1LL*INT_MAX,12)}));CHECK((cp::sparse_matrix_vector_multiply_mod<1>(composite,V{INT_MIN})==V{0}));
 auto cyclic=[](const V&v){V out(v.size());for(std::size_t i=0;i<v.size();++i)out[i]=v[(i+1)%v.size()];return out;};
 V rhs(128);rhs[0]=1;const auto cyclic_solution=cp::wiedemann_solve(128,cyclic,rhs);CHECK(cyclic_solution&&cyclic(*cyclic_solution)==rhs);
 V annihilator(129);annihilator[0]=998244352;annihilator[128]=1;auto far=cp::black_box_power_apply(128,cyclic,rhs,UINT64_MAX,annihilator);V wanted(128);wanted[(128-UINT64_MAX%128)%128]=1;CHECK(far==wanted);
 M diagonal(16,16);V expected_poly{1};for(int i=0;i<16;++i){diagonal(i,i)=i;expected_poly=mul(expected_poly,V{norm(-i),1});}CHECK(cp::matrix_minimal_polynomial_mod(diagonal)==expected_poly);
 std::cout<<"B set/algebra: all partitions, small-characteristic SPS, scalar and full minimal polynomials, verified solves and Woodbury passed\n";
}
