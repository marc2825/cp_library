#pragma once
#include <functional>
#include "treap.hpp"
namespace cp {
// 任意の新規キーを受ける順位付き集合。重複挿入ではfalse。
template<class Key,class Compare=std::less<Key>> class OrderedSet:public Treap<Key,Compare> {
    using Base=Treap<Key,Compare>;
public:
    using Base::Base;
    bool insert(const Key& x) { if(Base::count(x)) return false; Base::insert(x); return true; }
};
}
