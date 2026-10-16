#pragma once
#include "../range_query/segtree.hpp"

// Compatibility: prefer cp::Segtree from range_query/segtree.hpp.
template <class S, S (*op)(S, S), S (*e)()>
using segtree = cp::Segtree<S, op, e>;
