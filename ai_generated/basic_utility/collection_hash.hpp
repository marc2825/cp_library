#pragma once
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include "seeded_hash.hpp"

namespace cp {
struct CollectionFingerprint {
    std::uint64_t seed, size, first, second;
    friend bool operator==(const CollectionFingerprint& a, const CollectionFingerprint& b) {
        return std::tie(a.seed, a.size, a.first, a.second) == std::tie(b.seed, b.size, b.first, b.second);
    }
    friend bool operator!=(const CollectionFingerprint& a, const CollectionFingerprint& b) { return !(a == b); }
};
// Compare-equivalent keys must have equal SeededHash values. Fingerprints are probabilistic.
template<class Key, bool Multi, class Compare = std::less<Key>> class CollectionHash {
    std::map<Key, std::uint64_t, Compare> counts_;
    SeededHash first_, second_;
    std::uint64_t size_ = 0, a_ = 0, b_ = 0;
    void change(const Key& key, bool add) {
        auto a = first_.hash64(key), b = second_.hash64(key);
        if constexpr (Multi) { a_ += add ? a : -a; b_ += add ? b : -b; }
        else { a_ ^= a; b_ ^= b; }
    }
public:
    explicit CollectionHash(std::uint64_t seed = SeededHash::random_seed(), Compare compare = Compare())
        : counts_(compare), first_(seed), second_(seed ^ UINT64_C(0x9e3779b97f4a7c15)) {}
    std::uint64_t size() const { return size_; }
    std::size_t distinct_size() const { return counts_.size(); }
    std::uint64_t count(const Key& key) const { auto it = counts_.find(key); return it == counts_.end() ? 0 : it->second; }
    bool insert(const Key& key) {
        if constexpr (!Multi) if (count(key)) return false;
        if (size_ == std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("collection size overflow");
        ++counts_[key]; ++size_; change(key, true); return true;
    }
    bool erase(const Key& key) {
        auto it = counts_.find(key); if (it == counts_.end()) return false;
        change(key, false); --size_;
        if (--it->second == 0) counts_.erase(it);
        return true;
    }
    CollectionFingerprint fingerprint() const { return {first_.seed(), size_, a_, b_}; }
    void clear() { counts_.clear(); size_ = a_ = b_ = 0; }
};
template<class Key, class Compare = std::less<Key>> using SetHash = CollectionHash<Key, false, Compare>;
template<class Key, class Compare = std::less<Key>> using MultisetHash = CollectionHash<Key, true, Compare>;
} // namespace cp
