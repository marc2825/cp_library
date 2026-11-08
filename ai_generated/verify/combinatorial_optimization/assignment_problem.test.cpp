#define PROBLEM "https://judge.yosupo.jp/problem/assignment"
#include <iostream>
#include <vector>
#include "combinatorial_optimization/assignment_problem.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int n;std::cin>>n;cp::AssignmentProblem p(n,n);for(int i=0;i<n;++i)for(int j=0;j<n;++j){long long x;std::cin>>x;p.set_cost(i,j,x);}auto r=p.solve();std::cout<<r->cost<<'\n';for(auto j:r->row_to_column)std::cout<<j<<' ';std::cout<<'\n';}
