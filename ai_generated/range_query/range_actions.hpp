#pragma once
#include <optional>
#include <vector>
#include "lazy_segtree.hpp"

namespace cp {
template<class T> struct RangeSum { T sum; long long length; };
template<class T> struct SumMonoid {
    using S=RangeSum<T>;
    static S op(S a,S b) { return {a.sum+b.sum,a.length+b.length}; }
    static S e() { return {T(0),0}; }
    static S zero(long long l,long long r) { return {T(0),r-l}; }
    static std::vector<S> values(const std::vector<T>& a) {
        std::vector<S> v; for (const T& x:a) v.push_back({x,1}); return v;
    }
};
template<class T> struct AddSumAction:SumMonoid<T> {
    using S=RangeSum<T>; using F=T;
    static S mapping(F f,S x) { return {x.sum+f*T(x.length),x.length}; }
    static F composition(F f,F g) { return f+g; }
    static F id() { return T(0); }
};
template<class T> struct AssignSumAction:SumMonoid<T> {
    using S=RangeSum<T>; using F=std::optional<T>;
    static S mapping(F f,S x) { return {f?*f*T(x.length):x.sum,x.length}; }
    static F composition(F f,F g) { return f?f:g; }
    static F id() { return std::nullopt; }
};
template<class T> struct AffineSumAction:SumMonoid<T> {
    using S=RangeSum<T>; struct F { T a,b; };
    static S mapping(F f,S x) { return {f.a*x.sum+f.b*T(x.length),x.length}; }
    static F composition(F f,F g) { return {f.a*g.a,f.a*g.b+f.b}; }
    static F id() { return {T(1),T(0)}; }
};
template<class Action> using ActionLazySegtree=LazySegtree<typename Action::S,Action::op,Action::e,
    typename Action::F,Action::mapping,Action::composition,Action::id>;
} // namespace cp
