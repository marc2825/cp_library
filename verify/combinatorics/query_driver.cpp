// Python任意精度整数で、64ビット法と全APIの条件を比較する。
#include <iostream>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "combinatorics/all.hpp"
struct Query { char operation; long long n, k; };
template <class Table> void answer(const Table& table,const std::vector<Query>& queries) {
    for(auto q:queries)try {
        if(q.operation=='C')std::cout<<cp::binomial(table,q.n,q.k);
        else if(q.operation=='H')std::cout<<cp::multiset_combination(table,q.n,q.k);
        else if(q.operation=='A')std::cout<<cp::catalan(table,q.n);
        else {
            if constexpr(std::is_same_v<Table,cp::FactorialTable>){
                if(q.operation=='F')std::cout<<table.factorial(q.n);
                else if(q.operation=='I')std::cout<<table.inverse_factorial(q.n);
                else std::cout<<cp::permutations(table,q.n,q.k);
            }
        }
        std::cout<<'\n';
    }catch(const std::out_of_range&){std::cout<<"RANGE\n";}
}
int main(){char kind;int bound,q;long long modulus;while(std::cin>>kind>>bound>>modulus>>q){std::vector<Query> queries(q);for(auto& query:queries)std::cin>>query.operation>>query.n>>query.k;try{if(kind=='F')answer(cp::FactorialTable(bound,modulus),queries);else answer(cp::PascalBinomial(bound,modulus),queries);}catch(const std::domain_error&){std::cout<<"DOMAIN\n";}}}
