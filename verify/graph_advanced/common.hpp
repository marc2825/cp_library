#pragma once
#include <algorithm>
#include <cstdlib>
#include <climits>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <vector>
#define CHECK(x) do { if(!(x)) { std::cerr<<__FILE__<<':'<<__LINE__<<": "<<#x<<'\n'; std::abort(); } } while(false)
template<class E,class F> void throws(F f) { bool caught=false; try { f(); } catch(const E&) { caught=true; } CHECK(caught); }
