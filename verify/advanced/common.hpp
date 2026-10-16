#pragma once
#include <stdexcept>
#include <string>
inline void require(bool condition,const std::string& message) { if(!condition) throw std::runtime_error(message); }
template<class E,class F> void rejects(F f) { bool caught=false; try { f(); } catch(const E&) { caught=true; } require(caught,"expected exception"); }
