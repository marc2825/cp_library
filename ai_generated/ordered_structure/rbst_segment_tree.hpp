#pragma once
#include "persistent_lazy_rbst.hpp"

namespace cp {
namespace detail {
struct RBSTIdentityAction {};
template<class S> S rbst_identity_mapping(RBSTIdentityAction,const S& x) { return x; }
inline RBSTIdentityAction rbst_identity_composition(RBSTIdentityAction,RBSTIdentityAction) { return {}; }
inline RBSTIdentityAction rbst_identity_action() { return {}; }
} // namespace detail
// 動的列の点更新・区間積・反転・split/concat。期待O(log n)、コピーO(1)。
template<class S,auto op,auto e>
using RBSTSegmentTree=PersistentLazyRBST<S,op,e,detail::RBSTIdentityAction,
    detail::rbst_identity_mapping<S>,detail::rbst_identity_composition,detail::rbst_identity_action>;
} // namespace cp
