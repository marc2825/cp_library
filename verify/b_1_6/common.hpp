#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <vector>
#define CHECK(...) do { if(!(__VA_ARGS__)) { std::cerr<<__FILE__<<":"<<__LINE__<<" "<<#__VA_ARGS__<<"\n"; std::abort(); } } while(false)
inline std::mt19937_64 rng(3162026);
inline int pick(int n) { return int(rng()%unsigned(n)); }
inline long long sum_op(long long a,long long b) { return a+b; }
inline long long sum_e() { return 0; }
inline std::string cat(std::string a,std::string b) { return a+b; }
inline std::string empty_string() { return {}; }
