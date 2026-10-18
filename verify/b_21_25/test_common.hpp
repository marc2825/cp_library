#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
#define CHECK(...) do{if(!(__VA_ARGS__)){std::cerr<<__FILE__<<':'<<__LINE__<<": "<<#__VA_ARGS__<<'\n';std::abort();}}while(false)
template<class Exception=std::exception,class Function>void rejects(Function f){bool caught=false;try{f();}catch(const Exception&){caught=true;}CHECK(caught);}
inline bool close(long double a,long double b,long double eps=1e-8L){return std::abs(a-b)<=eps*std::max({1.L,std::abs(a),std::abs(b)});}
