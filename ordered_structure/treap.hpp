#pragma once
#include <functional>
#include "detail/ordered_tree.hpp"
namespace cp {
// 動的キー多重集合。順位・k番目・挿入・一個削除は期待O(log n)。
template<class Key,class Compare=std::less<Key>> using Treap=detail::OrderedTree<Key,Compare,false>;
}
