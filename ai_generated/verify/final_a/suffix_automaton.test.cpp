#define PROBLEM "https://judge.yosupo.jp/problem/number_of_substrings"
#include <iostream>
#include <string>
#include "string_algorithm/suffix_automaton.hpp"
int main(){std::string s;std::cin>>s;std::cout<<cp::SuffixAutomaton(s).distinct_substrings()<<'\n';}
