#pragma once
// 説明: docs/ordered_structure/radix_heap.md
#include <type_traits>
#include "../data_structure/radix_heap_iwiwi.hpp"
namespace cp {
// 元実装を共有。符号なし32/64bit、pushは直前pop以上のkey。
template<class Key,class Value> class RadixHeap : public radix_heap::pair_radix_heap<Key,Value> {
    static_assert(std::is_unsigned<Key>::value && (sizeof(Key)==4 || sizeof(Key)==8),"unsigned 32/64-bit key required");
};
}
