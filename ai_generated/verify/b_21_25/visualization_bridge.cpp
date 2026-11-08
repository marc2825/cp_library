#include "../../string_algorithm/all.hpp"
#include "../../computational_geometry/all.hpp"
#include "../../combinatorial_optimization/all.hpp"
#include "../../game_theory/all.hpp"
#include "../../heuristic_search/all.hpp"
#include <iomanip>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>
using V=std::vector<int>;namespace E=cp::geometry::exact;namespace F=cp::geometry::floating;
template<class T,std::enable_if_t<std::is_arithmetic_v<T>,int> =0>void out(T x){std::cout<<x;}
void out(const std::string&s){std::cout<<'"'<<s<<'"';}
void out(const E::Point&p){std::cout<<'['<<p.x<<','<<p.y<<']';}
void out(const F::Point&p){std::cout<<'['<<p.x<<','<<p.y<<']';}
template<class T>void out(const std::vector<T>&v){std::cout<<'[';for(std::size_t i=0;i<v.size();++i){if(i)std::cout<<',';out(v[i]);}std::cout<<']';}
void start(const char*mode){std::cout<<'\n'<<mode<<' ';}
int main(){std::cout<<std::setprecision(20)<<std::boolalpha;
 start("runs_enumeration");{auto r=cp::runs_enumeration("ababaabababa");std::vector<V>v;for(auto run:r)v.push_back({run.period,run.left,run.right});out(v);}
 start("suffix_tree");out(cp::SuffixTree("banana").occurrences("ana"));
 start("wildcard_pattern_matching");out(cp::wildcard_pattern_matching("abacaba","a?a"));
 start("dynamic_rolling_hash");{cp::DynamicRollingHash s("abc",257);s.insert(1,'x');s.reverse(0,4);s.set(0,'z');std::cout<<'[';out(s.str());std::cout<<',';auto hash=s.get(0,s.size());out(std::vector<std::uint32_t>(hash.begin(),hash.end()));std::cout<<']';}
 start("minimum_rotation");out(cp::minimum_rotation("baca"));
 start("multi_string_compare");{cp::MultiStringCompare s({"banana","band","ananas"});out(std::vector<V>{{s.lcp(0,0,6,1,0,4),s.compare(0,0,6,1,0,4)},{s.lcp(0,1,4,2,0,3),s.compare(0,1,4,2,0,3)}});}
 start("palindrome_partition");out(cp::palindrome_partition("abacdc").prefix_minimum);
 start("minimum_enclosing_circle");{auto c=F::minimum_enclosing_circle({{0,0},{4,0},{0,3},{1,1}});out(std::vector<long double>{c->center.x,c->center.y,c->radius});}
 start("circle_intersection");{F::Circle a({0,0},2),b({2,0},2);auto cc=F::circle_circle_intersection(a,b),cl=F::circle_line_intersection(a,F::Line({-3,0},{3,0}));std::cout<<'[';out(cc.points);std::cout<<',';out(cl.points);std::cout<<',';out(cc.coincident);std::cout<<']';}
 start("convex_layers");out(E::convex_layers({{0,0},{4,0},{4,4},{0,4},{1,1},{3,1},{3,3},{1,3},{2,2}}));
 start("minkowski_sum");out(E::minkowski_sum({{0,0},{2,0},{0,2}},{{1,1},{2,1},{1,2}}));
 start("farthest_pair");out(static_cast<long long>(E::farthest_pair({{0,0},{3,4},{1,1},{2,-1}})->squared_distance));
 start("manhattan_mst");out(cp::manhattan_mst({{0,0},{1,0},{1,2},{3,3}}).cost);
 start("pick");{const auto c=E::lattice_polygon_count({{0,0},{3,0},{3,2},{0,2}});out(std::vector<long long>{static_cast<long long>(c.interior),static_cast<long long>(c.boundary)});}
 start("half_plane_intersection");{auto p=E::half_plane_intersection({E::Line({0,0},{1,0}),E::Line({2,0},{2,1}),E::Line({2,2},{1,2})},{-2,-2},{3,3});out(E::polygon_area(p).approximate());}
 start("matroid_intersection");out(cp::matroid_intersection(5,cp::GraphicMatroid(4,{{0,1},{1,2},{2,0},{2,3},{0,3}}),cp::PartitionMatroid({0,0,1,1,2},{1,1,1})).selected);
 start("graphic_matroid");out(cp::GraphicMatroid(3,{{0,1},{1,2},{2,0}}).independent({0,1,2}));
 start("partition_matroid");out(cp::PartitionMatroid({0,0,1,1},{1,2}).independent({0,2,3}));
 start("maximum_weight_independent_set");{cp::Graph<int>g(4);g.add_edge(0,1);g.add_edge(1,2);g.add_edge(2,3);out(static_cast<long long>(cp::maximum_weight_independent_set(g,{2,5,3,4}).weight));}
 start("linear_matroid_parity");out(cp::linear_matroid_parity_exact<17>({{{1,0,0,0},{0,1,0,0}},{{0,0,1,0},{0,0,0,1}},{{1,0,0,0},{0,0,1,0}}}));
 start("nimber");std::cout<<"[\""<<(3^5)<<"\",\""<<cp::nim_product(3,5)<<"\",\""<<cp::nimber_inverse(3)<<"\"]";
 start("graph_game");{auto r=cp::graph_game({{1},{},{2},{0,2}});V status;for(auto x:r.status)status.push_back(x==cp::GraphGameStatus::win?1:x==cp::GraphGameStatus::lose?-1:0);out(status);}
 start("partizan_game");{cp::PartizanGameArena arena;const int star=arena.make({0},{0});out(arena.compare(arena.add(star,star),0)==cp::PartizanOrder::equal?std::string("equal"):std::string("unexpected"));}
 start("multi_start_sa");{cp::AnnealingOptions o;o.max_iterations=10;o.start_temperature=10;o.end_temperature=1;auto r=cp::multi_start_sa(3,[](std::size_t,cp::Random&r){return r.integer(-8,8);},[](int x){return x*x;},[](int x,cp::Random&r){return x+r.integer(-2,2);},123,o);V scores;for(const auto&run:r.runs)scores.push_back(run.best_score);std::cout<<'[';out(scores);std::cout<<',';out(r.best_run_index);std::cout<<',';out(r.best_run().best_score);std::cout<<']';}
 start("temperature_control");{cp::AcceptanceTemperatureController c(2,0.5L,1);std::vector<long double>history;for(const auto&v:std::vector<V>{{0,0,0,0},{1,1,1,0}}){for(int x:v)c.observe(true,x);history.push_back(c.update());}std::cout<<'[';out(cp::calibrate_annealing_temperature({1,2,4},0.5L));std::cout<<',';out(history);std::cout<<']';}
 start("parameter_tuning");{const auto r=cp::tune_parameters(V{1,2,4},std::vector<std::uint64_t>{1,3,8},[](int c,std::uint64_t s){return c*c+int(s%7);});std::cout<<'[';out(r.best_index);std::cout<<',';out(r.mean_scores);std::cout<<',';out(r.scores);std::cout<<']';}
 start("multi_armed_bandit");{cp::MultiArmedBandit b(2);const std::vector<V> rewards{{0,1,0},{1,1,0}};for(int i=0;i<14;++i){auto arm=b.select_ucb1();const auto reward=rewards[arm][b.pulls()[arm]%rewards[arm].size()];b.update(arm,reward);}std::cout<<'[';out(b.pulls());std::cout<<',';out(b.means());std::cout<<']';}
 start("neighborhood_manager");{cp::NeighborhoodManager m({1,3});cp::Random random(321);cp::AnnealingOptions o;o.max_iterations=12;o.start_temperature=5;o.end_temperature=0.5L;auto r=cp::managed_simulated_annealing(5,[](int x){return x*x;},[](int x,std::size_t id,cp::Random&r){const int d=int(id)+1;return x+(r.uniform(2)?-d:d);},m,random,o);std::vector<V>stats;for(auto s:m.statistics())stats.push_back({int(s.attempts),int(s.accepted),int(s.improved)});std::cout<<'[';out(r.best_score);std::cout<<',';out(r.current_score);std::cout<<',';out(stats);std::cout<<']';}
 std::cout<<'\n';
}
