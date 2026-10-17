#pragma once
#include <functional>
#include "detail/ordered_tree.hpp"
namespace cp {
// ランダム挿入・サイズ比例結合による多重集合。固定seedを指定可。
template<class Key,class Compare=std::less<Key>> using RBST=detail::OrderedTree<Key,Compare,true>;
}
