#include <cstdint>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "game_theory/all.hpp"
void print(std::ostream&out,std::optional<cp::NimMove> move){if(!move)out<<"NONE";else out<<move->pile<<' '<<move->remaining<<' '<<move->removed;}
int main(){std::string op;while(std::cin>>op){std::ostringstream out;try{
 if(op=="N"){std::size_t n;std::cin>>n;std::vector<std::uint64_t>a(n);for(auto&x:a)std::cin>>x;out<<cp::nim_sum(a)<<' '<<cp::nim_winning(a)<<' ';print(out,cp::nim_winning_move(a));out<<" | "<<cp::misere_nim_winning(a)<<' ';print(out,cp::misere_nim_winning_move(a));}
 else if(op=="S"){std::size_t n;std::cin>>n;std::vector<long long>a(n);for(auto&x:a)std::cin>>x;out<<cp::nim_sum(a)<<' '<<cp::misere_nim_winning(a);}
 else if(op=="X"){std::size_t n;std::cin>>n;std::vector<int>a(n);for(auto&x:a)std::cin>>x;out<<cp::grundy_mex(a);}
 else if(op=="D"){int n,m,q;std::cin>>n>>m>>q;std::vector<std::vector<int>>moves(n);while(m--){int a,b;std::cin>>a>>b;moves.at(a).push_back(b);}std::vector<std::vector<int>>queries(q);for(auto&positions:queries){int k;std::cin>>k;positions.resize(k);for(auto&v:positions)std::cin>>v;}cp::DAGGame game(std::move(moves));for(int g:game.grundy())out<<g<<' ';out<<'|';for(int v:game.topological_order())out<<' '<<v;for(const auto&positions:queries){out<<" | "<<game.nim_sum(positions)<<' '<<game.winning(positions)<<' ';auto move=game.winning_move(positions);if(move)out<<move->component<<' '<<move->from<<' '<<move->to;else out<<"NONE";}}
 }catch(const std::invalid_argument&){out.str("");out<<"INVALID";}catch(const std::out_of_range&){out.str("");out<<"RANGE";}std::cout<<out.str()<<'\n';}}
