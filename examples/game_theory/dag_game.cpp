#include <iostream>
#include <vector>
#include "game_theory/dag_game.hpp"
int main(){
    cp::Graph<int> graph(4,cp::GraphKind::Directed);
    graph.add_edge(1,0);graph.add_edge(2,0);graph.add_edge(2,1);
    graph.add_edge(3,1);graph.add_edge(3,2);
    cp::DAGGame game(graph);
    std::vector<int> tokens{1,2}; // Independent tokens; one moves each turn.
    std::cout<<game.nim_sum(tokens)<<' '<<game.winning(tokens)<<'\n';
    if(auto m=game.winning_move(tokens))
        std::cout<<m->component<<' '<<m->from<<' '<<m->to<<'\n';
}
