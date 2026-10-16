#pragma once
#include <optional>

// Helpers for std::set/multiset/map. Order is the container's comparator order.
// Missing predecessor/successor returns end(); inclusive=false excludes equal keys.
// Docs: docs/ordered_structure/ordered_associative.md
namespace cp {

template <class Container, class Key>
auto predecessor(Container& values, const Key& key, bool inclusive = false)
    -> decltype(values.lower_bound(key)) {
    auto it = inclusive ? values.upper_bound(key) : values.lower_bound(key);
    if (it == values.begin()) return values.end();
    return --it;
}

template <class Container, class Key>
auto successor(Container& values, const Key& key, bool inclusive = false)
    -> decltype(values.lower_bound(key)) {
    return inclusive ? values.lower_bound(key) : values.upper_bound(key);
}

template <class Container, class Key>
bool erase_one(Container& values, const Key& key) {
    auto it = values.find(key);
    if (it == values.end()) return false;
    values.erase(it);
    return true;
}

template <class Container>
std::optional<typename Container::value_type> pop_first(Container& values) {
    if (values.empty()) return std::nullopt;
    auto it = values.begin();
    std::optional<typename Container::value_type> result(*it);
    values.erase(it);
    return result;
}

template <class Container>
std::optional<typename Container::value_type> pop_last(Container& values) {
    if (values.empty()) return std::nullopt;
    auto it = values.end();
    --it;
    std::optional<typename Container::value_type> result(*it);
    values.erase(it);
    return result;
}

}  // namespace cp
