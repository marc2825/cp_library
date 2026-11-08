#include "../../convolution/all.hpp"
#include "../../fps/all.hpp"
#include "../../linear_recurrence/all.hpp"
#include "../../set_function/all.hpp"
#include "../../linear_algebra/all.hpp"
#include <iostream>
#include <vector>
using V=std::vector<int>;using M=cp::Matrix<int>;
void out(int x){std::cout<<x;}
template<class T>void out(const std::vector<T>& a){std::cout<<'[';for(std::size_t i=0;i<a.size();++i){if(i)std::cout<<',';out(a[i]);}std::cout<<']';}
void out(const M&a){std::cout<<'[';for(std::size_t i=0;i<a.rows();++i){if(i)std::cout<<',';std::cout<<'[';for(std::size_t j=0;j<a.cols();++j){if(j)std::cout<<',';out(a(i,j));}std::cout<<']';}std::cout<<']';}
int main(){
{std::cout<<"relaxed_convolution ";
cp::RelaxedConvolution<97,5> f(6);V c;V a{1,2,3,4,5,6},b{2,1,0,3,1,4};for(std::size_t i=0;i<a.size();++i)c.push_back(f.append(a[i],b[i]));out(c);std::cout<<"\n";}
{std::cout<<"gcd_lcm_convolution ";
out(std::vector<V>{cp::gcd_convolution(V{0,1,2,3,4,5},V{0,3,1,2,0,1}),cp::lcm_convolution(V{0,1,2,3,4,5},V{0,3,1,2,0,1})});std::cout<<"\n";}
{std::cout<<"min_plus_convolution ";
out(cp::min_plus_convolution_convex_convex(V{0,1,4,9},V{0,2,7,15}));std::cout<<"\n";}
{std::cout<<"multidimensional_convolution ";
auto c=cp::multidimensional_convolution<97,5>(V{1,2,3,4},{2,2},V{1,2,1,0},{2,2});out(c.coefficients);std::cout<<"\n";}
{std::cout<<"convolution_large ";
out(cp::convolution_large<97,5>(V{1,2,3,4,5,6},V{2,1,3,1,2},2));std::cout<<"\n";}
{std::cout<<"middle_product ";
out(cp::middle_product<97,5>(V{1,2,3,4,5},V{2,1,3}));std::cout<<"\n";}
{std::cout<<"chirp_z ";
out(cp::chirp_z<97,5>(V{1,2,1},1,2,5));std::cout<<"\n";}
{std::cout<<"composition ";
out(cp::fps_compose<97,5>(V{1,2,3,1},V{1,1,2},6));std::cout<<"\n";}
{std::cout<<"compositional_inverse ";
out(cp::fps_compositional_inverse<97,5>(V{0,1,1},8));std::cout<<"\n";}
{std::cout<<"sparse_fps ";
out(cp::sparse_fps_inv<97>(cp::SparseFPS{{0,1},{2,-1},{5,2}},9));std::cout<<"\n";}
{std::cout<<"online_fps ";
cp::OnlineFPSExp<97,5> f(8);V c;for(auto a:V{0,1,0,2,0,0,0,0})c.push_back(f.append(a));out(c);std::cout<<"\n";}
{std::cout<<"polynomial_gcd ";
out(cp::polynomial_gcd<97,5>(V{-1,0,1},V{1,-2,1}));std::cout<<"\n";}
{std::cout<<"polynomial_mod_pow ";
out(cp::polynomial_mod_pow<97,5>(V{0,1},13,V{-1,-1,1}));std::cout<<"\n";}
{std::cout<<"subproduct_tree ";
cp::SubproductTree<97,5> tree(V{1,2,3,4});out(std::vector<V>{tree.product(),tree.evaluate(V{1,0,1})});std::cout<<"\n";}
{std::cout<<"partial_fraction ";
auto c=cp::partial_fraction_decomposition<97,5>(V{1,2,0,1},V{1,3},{2,1});std::vector<V> r{c.polynomial_part};r.insert(r.end(),c.coefficients.begin(),c.coefficients.end());out(r);std::cout<<"\n";}
{std::cout<<"infer_nth ";
out(cp::linear_recurrence_from_prefix<97>(V{0,1,1,2,3,5,8,13},12));std::cout<<"\n";}
{std::cout<<"p_recursive ";
out(cp::p_recursive_prefix<97>(V{1},std::vector<V>{{-1,-1},{1}},9));std::cout<<"\n";}
{std::cout<<"egf ";
out(cp::egf_multiply<97,5>(V{1,1,1,1,1,1},V{1,1,1,1,1,1},6));std::cout<<"\n";}
{std::cout<<"coefficient_extraction ";
out(cp::generating_function_coefficients<97,5>(V{1},V{1,-1,-1},10));std::cout<<"\n";}
{std::cout<<"rational_generating_function ";
out(cp::RationalGeneratingFunction<97,5>(V{1,-1},V{1,-2,1}).prefix(10));std::cout<<"\n";}
{std::cout<<"polynomial_geometric_sum ";
out(cp::polynomial_geometric_sum<97,5>(V{0,1,1},2,8));std::cout<<"\n";}
{std::cout<<"set_power_series ";
out(cp::sps_exp<97>(V{0,1,1,2,1,2,2,3}));std::cout<<"\n";}
{std::cout<<"sps_composition ";
out(cp::sps_compose<97>(V{1,2,1,3},V{1,1,2,3,1,0,2,1}));std::cout<<"\n";}
{std::cout<<"set_partition ";
out(cp::set_partition_by_blocks<97>(V{0,1,1,1,1,1,1,1}));std::cout<<"\n";}
{std::cout<<"fast_subset_convolution ";
out(cp::subset_convolution_mod<97>(V{1,2,3,4,5,6,7,8},V{1,1,1,1,1,1,1,1}));std::cout<<"\n";}
{std::cout<<"sparse_matrix ";
cp::SparseMatrix<int> a(3,3,{{0,1,2},{0,1,3},{1,2,-1},{2,0,4}});out(cp::sparse_matrix_vector_multiply_mod<97>(a,V{1,2,3}));std::cout<<"\n";}
{std::cout<<"black_box ";
M a{{1,1},{1,0}};auto apply=[&](const V&v){V r(a.rows());for(std::size_t i=0;i<a.rows();++i)for(std::size_t j=0;j<a.cols();++j)r[i]=(r[i]+a(i,j)*v[j])%97;return r;};auto p=cp::matrix_minimal_polynomial_mod<97>(a);out(cp::black_box_power_apply<97>(a.rows(),apply,V{0,1},8,p));std::cout<<"\n";}
{std::cout<<"wiedemann ";
M a{{1,1},{1,0}};auto apply=[&](const V&v){V r(a.rows());for(std::size_t i=0;i<a.rows();++i)for(std::size_t j=0;j<a.cols();++j)r[i]=(r[i]+a(i,j)*v[j])%97;return r;};out(cp::wiedemann_polynomial<97>(a.rows(),apply,V{1,0},V{0,1}));std::cout<<"\n";}
{std::cout<<"minimal_polynomial ";
M a{{2,1,0},{0,2,0},{0,0,3}};out(cp::matrix_minimal_polynomial_mod<97>(a));std::cout<<"\n";}
{std::cout<<"matrix_tree ";
cp::Graph<int> g(4);g.add_edge(0,1,2);g.add_edge(1,2,3);g.add_edge(2,3,1);g.add_edge(3,0,2);g.add_edge(0,2,4);out(cp::count_weighted_spanning_trees<97>(g));std::cout<<"\n";}
{std::cout<<"low_rank_update ";
M a{{2,0},{0,1}},u{{1},{0}},v{{2},{1}};auto ai=cp::matrix_inverse_mod<97>(a);out(*cp::low_rank_inverse_update_mod<97>(*ai,u,v));std::cout<<"\n";}
}
