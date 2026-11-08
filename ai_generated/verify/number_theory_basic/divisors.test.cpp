#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ITP1_3_D"
#include <iostream>
#include "number_theory_basic/divisors.hpp"
int main(){long long a,b,c;std::cin>>a>>b>>c;int count=0;for(long long d:cp::divisors(c))count+=a<=d&&d<=b;std::cout<<count<<'\n';}
