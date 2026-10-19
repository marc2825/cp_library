#pragma once
#include <cstddef>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace cp {
// Typed block arena. IDs and references remain valid until clear/destruction.
template <class T, std::size_t BlockSize = 256> class Arena {
    static_assert(BlockSize > 0, "nonempty arena blocks required");
    static_assert(std::is_nothrow_destructible_v<T>, "nothrow destructor required");
    struct alignas(T) Slot { std::byte bytes[sizeof(T)]; };
    std::vector<std::unique_ptr<Slot[]>> blocks_;
    std::size_t size_ = 0;
    T* pointer(std::size_t id) const {
        return std::launder(reinterpret_cast<T*>(blocks_[id / BlockSize][id % BlockSize].bytes));
    }
public:
    using id_type = std::size_t;
    Arena() = default;
    Arena(const Arena&) = delete;
    Arena& operator=(const Arena&) = delete;
    Arena(Arena&&) = delete;
    Arena& operator=(Arena&&) = delete;
    ~Arena() { clear(); }
    template <class... Args> id_type emplace(Args&&... args) {
        if (size_ == capacity()) {
            if (blocks_.size() >= std::numeric_limits<std::size_t>::max() / BlockSize)
                throw std::length_error("Arena capacity exceeds size_t");
            blocks_.push_back(std::make_unique<Slot[]>(BlockSize));
        }
        void* storage = blocks_[size_ / BlockSize][size_ % BlockSize].bytes;
        ::new (storage) T(std::forward<Args>(args)...);
        return size_++;
    }
    T& operator[](id_type id) {
        if (id >= size_) throw std::out_of_range("Arena id out of range");
        return *pointer(id);
    }
    const T& operator[](id_type id) const {
        if (id >= size_) throw std::out_of_range("Arena id out of range");
        return *pointer(id);
    }
    std::size_t size() const { return size_; }
    std::size_t capacity() const { return blocks_.size() * BlockSize; }
    bool empty() const { return size_ == 0; }
    void clear() noexcept {
        while (size_) pointer(--size_)->~T();
        blocks_.clear();
    }
};
} // namespace cp
