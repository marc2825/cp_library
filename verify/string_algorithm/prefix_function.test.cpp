#define PROBLEM "https://cses.fi/problemset/task/1732"
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include "string_algorithm/prefix_function.hpp"
int main(){std::string s;std::cin>>s;auto p=cp::prefix_function(s);std::vector<int>borders;for(int k=p.empty()?0:p.back();k;k=p[k-1])borders.push_back(k);std::reverse(borders.begin(),borders.end());for(auto k:borders)std::cout<<k<<' ';std::cout<<'\n';}
