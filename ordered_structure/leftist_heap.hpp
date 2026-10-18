#pragma once
// 説明: docs/ordered_structure/leftist_heap.md
#include "detail/meldable_heap.hpp"
namespace cp {
template<class T,class Compare=std::less<T>> using LeftistHeap=detail::MeldableHeap<T,Compare,true>;
}
