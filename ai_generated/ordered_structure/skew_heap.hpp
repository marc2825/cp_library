#pragma once
// 説明: docs/ordered_structure/skew_heap.md
#include "detail/meldable_heap.hpp"
namespace cp {
template<class T,class Compare=std::less<T>> using SkewHeap=detail::MeldableHeap<T,Compare,false>;
}
