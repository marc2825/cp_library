#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "combinatorial_optimization/all.hpp"
int main(){std::string op;while(std::cin>>op){std::ostringstream out;try{
 if(op=="A"){int n,m,maximum;std::cin>>n>>m>>maximum;cp::AssignmentProblem p(n,m);for(int i=0;i<n;++i)for(int j=0;j<m;++j){std::string x;std::cin>>x;if(x!="x")p.set_cost(i,j,std::stoll(x));}auto r=p.solve(maximum?cp::AssignmentObjective::maximize:cp::AssignmentObjective::minimize);if(!r)out<<"NONE";else{out<<r->cost;for(auto j:r->row_to_column)out<<' '<<j;out<<" |";for(auto x:r->row_potential)out<<' '<<cp::optimization_to_string(x);out<<" |";for(auto x:r->column_potential)out<<' '<<cp::optimization_to_string(x);}}
 else if(op=="M"){int n,m;std::cin>>n>>m;cp::GeneralMatching g(n);while(m--){int u,v;std::cin>>u>>v;g.add_edge(u,v);}auto r=g.maximum_matching();out<<r.cardinality;for(int mate:r.mate)out<<' '<<mate;}
 else if(op=="F"){int n,m,q;std::cin>>n>>m>>q;cp::MinCostFlow<>g(n);while(m--){int u,v;long long cap,cost;std::cin>>u>>v>>cap>>cost;g.add_edge(u,v,cap,cost);}std::vector<long long>limits(q);for(auto&x:limits)std::cin>>x;for(auto limit:limits){auto r=g.flow(0,n-1,limit);out<<r.flow<<' '<<r.cost<<' ';}out<<'|';for(const auto&e:g.edges())out<<' '<<e.flow;g.reset_flow();const auto slope=g.slope(0,n-1);out<<" |";for(auto p:slope)out<<' '<<p.flow<<' '<<p.cost;}
 }catch(const std::overflow_error&){out.str("");out<<"OVERFLOW";}catch(const std::domain_error&){out.str("");out<<"NEGATIVE_CYCLE";}catch(const std::invalid_argument&){out.str("");out<<"INVALID";}std::cout<<out.str()<<'\n';}}
