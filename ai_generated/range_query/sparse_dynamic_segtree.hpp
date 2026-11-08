#pragma once
// 説明: docs/range_query/sparse_dynamic_segtree.md
// 未作成節点を単位元とする疎な既存DynamicSegtreeを共有。
#include "dynamic_segtree.hpp"
namespace cp {
template<class S,auto op,auto e> using SparseDynamicSegtree=DynamicSegtree<S,op,e>;
}
