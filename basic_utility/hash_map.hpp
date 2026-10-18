#pragma once
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>
#include "seeded_hash.hpp"

namespace cp {
// Open addressing with linear probing and tombstones; live load <= 1/2.
// find returns a value pointer. Rehash invalidates all pointers/references.
template <class Key, class T, class Hash = SeededHash, class Equal = std::equal_to<Key>>
class HashMap {
    static_assert(std::is_copy_constructible_v<Key> && std::is_copy_constructible_v<T>, "copyable key/value required");
    static_assert(std::is_nothrow_swappable_v<Hash> && std::is_nothrow_swappable_v<Equal>, "nothrow-swappable policies required");
    struct Slot {
        std::optional<std::pair<Key, T>> entry;
        std::size_t hash = 0;
        bool deleted = false;
    };
    Hash hash_;
    Equal equal_;
    std::vector<Slot> slots_;
    std::size_t size_ = 0, used_ = 0;
    static constexpr std::size_t missing = std::numeric_limits<std::size_t>::max();
    std::size_t locate(const Key& key, std::size_t hash) const {
        if (slots_.empty()) return missing;
        std::size_t at = hash & (slots_.size() - 1);
        for (std::size_t seen = 0; seen < slots_.size(); ++seen) {
            const auto& slot = slots_[at];
            if (slot.entry) {
                if (slot.hash == hash && equal_(slot.entry->first, key)) return at;
            } else if (!slot.deleted) return missing;
            at = (at + 1) & (slots_.size() - 1);
        }
        return missing;
    }
    static std::size_t vacancy(const std::vector<Slot>& slots, std::size_t hash) {
        std::size_t at = hash & (slots.size() - 1);
        while (slots[at].entry) at = (at + 1) & (slots.size() - 1);
        return at;
    }
    void prepare_insert() {
        if (slots_.empty()) rehash(8);
        else if (size_ == slots_.size() / 2) {
            if (slots_.size() > slots_.max_size() / 2) throw std::length_error("HashMap capacity exceeds limit");
            rehash(slots_.size() * 2);
        } else if (used_ >= slots_.size() - slots_.size() / 4) rehash(slots_.size());
    }
public:
    explicit HashMap(Hash hash = Hash{}, Equal equal = Equal{}) : hash_(std::move(hash)), equal_(std::move(equal)) {}
    HashMap(const HashMap&) = default;
    HashMap(HashMap&& other)
        : hash_(other.hash_), equal_(other.equal_), slots_(std::move(other.slots_)),
          size_(std::exchange(other.size_, 0)), used_(std::exchange(other.used_, 0)) { other.slots_.clear(); }
    HashMap& operator=(HashMap other) { swap(other); return *this; }
    void swap(HashMap& other) noexcept {
        using std::swap;
        swap(hash_, other.hash_); swap(equal_, other.equal_); slots_.swap(other.slots_);
        swap(size_, other.size_); swap(used_, other.used_);
    }
    friend void swap(HashMap& a, HashMap& b) noexcept { a.swap(b); }
    std::size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    std::size_t capacity() const { return slots_.size(); }
    double load_factor() const { return capacity() ? double(size_) / capacity() : 0; }
    Hash hash_function() const { return hash_; }
    const T* find(const Key& key) const {
        const auto at = locate(key, hash_(key));
        return at == missing ? nullptr : &slots_[at].entry->second;
    }
    T* find(const Key& key) { return const_cast<T*>(std::as_const(*this).find(key)); }
    bool contains(const Key& key) const { return find(key) != nullptr; }
    const T& at(const Key& key) const {
        const auto* value = find(key);
        if (!value) throw std::out_of_range("HashMap key not found");
        return *value;
    }
    T& at(const Key& key) { return const_cast<T&>(std::as_const(*this).at(key)); }
    template <class... Args> std::pair<T*, bool> try_emplace(const Key& key, Args&&... args) {
        const std::size_t hash = hash_(key);
        const auto old = locate(key, hash);
        if (old != missing) return {&slots_[old].entry->second, false};
        // Own the arguments before rehash: they may refer to existing values.
        std::pair<Key, T> candidate(std::piecewise_construct, std::forward_as_tuple(key),
                                    std::forward_as_tuple(std::forward<Args>(args)...));
        prepare_insert();
        auto& slot = slots_[vacancy(slots_, hash)];
        slot.entry.emplace(std::move(candidate));
        if (!slot.deleted) ++used_;
        slot.deleted = false; slot.hash = hash; ++size_;
        return {&slot.entry->second, true};
    }
    std::pair<T*, bool> insert(const Key& key, const T& value) { return try_emplace(key, value); }
    std::pair<T*, bool> insert_or_assign(const Key& key, const T& value) {
        if (auto* old = find(key)) { *old = value; return {old, false}; }
        return try_emplace(key, value);
    }
    T& operator[](const Key& key) { return *try_emplace(key).first; }
    bool erase(const Key& key) {
        const auto at = locate(key, hash_(key));
        if (at == missing) return false;
        slots_[at].entry.reset(); slots_[at].deleted = true; --size_;
        return true;
    }
    void clear() {
        for (auto& slot : slots_) { slot.entry.reset(); slot.deleted = false; }
        size_ = used_ = 0;
    }
    void reserve(std::size_t elements) {
        if (elements <= capacity() / 2) return;
        if (elements > slots_.max_size() / 2) throw std::length_error("HashMap reservation exceeds limit");
        rehash(elements * 2);
    }
    void rehash(std::size_t requested) {
        std::size_t count = 8;
        while (count < requested || count / 2 < size_) {
            if (count > slots_.max_size() / 2) throw std::length_error("HashMap capacity exceeds limit");
            count *= 2;
        }
        std::vector<Slot> next(count);
        for (const auto& old : slots_) if (old.entry) {
            auto& slot = next[vacancy(next, old.hash)];
            slot.entry.emplace(*old.entry); slot.hash = old.hash;
        }
        slots_.swap(next); used_ = size_;
    }
    template <class F> void for_each(F&& visit) {
        for (auto& slot : slots_) if (slot.entry) std::invoke(visit, std::as_const(slot.entry->first), slot.entry->second);
    }
    template <class F> void for_each(F&& visit) const {
        for (const auto& slot : slots_) if (slot.entry) std::invoke(visit, slot.entry->first, slot.entry->second);
    }
};
} // namespace cp
