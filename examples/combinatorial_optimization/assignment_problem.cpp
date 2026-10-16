#include <iostream>
#include <vector>
#include "combinatorial_optimization/assignment_problem.hpp"
int main(){
    cp::AssignmentProblem problem(2,3); // Initially all cells are forbidden.
    problem.set_cost(0,0,5);problem.set_cost(0,1,8);
    problem.set_cost(1,0,7);problem.set_cost(1,2,4);
    std::cout<<problem.solve()->cost<<'\n';
    std::cout<<problem.solve(cp::AssignmentObjective::maximize)->cost<<'\n';
    problem.forbid(0,0);problem.forbid(0,1);
    std::cout<<bool(problem.solve())<<'\n';
}
