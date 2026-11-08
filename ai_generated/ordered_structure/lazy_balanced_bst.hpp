#pragma once
// 説明: docs/ordered_structure/lazy_balanced_bst.md
// 区間作用・反転・挿入/削除に対応する既存の平衡BSTを共有。
#include "lazy_reversible_treap.hpp"
namespace cp {
template<class S,auto op,auto e,class F,auto mapping,auto composition,auto id>
using LazyBalancedBST=LazyReversibleTreap<S,op,e,F,mapping,composition,id>;
}
