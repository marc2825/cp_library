#pragma once
#include "random.hpp"
#include <climits>
#include <optional>
#include <vector>
namespace cp {
// Fixed universe [0,n). Element IDs are stable; dense indices change on erase.
class IndexSet {
    std::vector<int> dense_, position_;
    void check(int x) const {
        if (x < 0 || static_cast<std::size_t>(x) >= position_.size())
            throw std::out_of_range("IndexSet element");
    }

  public:
    explicit IndexSet(int universe) {
        if (universe < 0)
            throw std::invalid_argument("negative IndexSet universe");
        position_.assign(universe, -1);
        dense_.reserve(universe);
    }
    IndexSet(const IndexSet &other) : dense_(other.dense_), position_(other.position_) {
        dense_.reserve(position_.size());
    }
    IndexSet(IndexSet &&other) noexcept { swap(other); }
    IndexSet &operator=(IndexSet other) noexcept {
        swap(other);
        return *this;
    }
    void swap(IndexSet &other) noexcept {
        dense_.swap(other.dense_);
        position_.swap(other.position_);
    }
    std::size_t universe_size() const { return position_.size(); }
    std::size_t size() const { return dense_.size(); }
    bool empty() const { return dense_.empty(); }
    bool contains(int x) const {
        check(x);
        return position_[x] >= 0;
    }
    bool insert(int x) {
        check(x);
        if (position_[x] >= 0)
            return false;
        dense_.push_back(x);
        position_[x] = static_cast<int>(dense_.size() - 1);
        return true;
    }
    bool erase(int x) {
        check(x);
        const int p = position_[x];
        if (p < 0)
            return false;
        dense_[p] = dense_.back();
        position_[dense_.back()] = p;
        dense_.pop_back();
        position_[x] = -1;
        return true;
    }
    void clear() {
        for (int x : dense_)
            position_[x] = -1;
        dense_.clear();
    }
    const std::vector<int> &values() const { return dense_; }
    int at(std::size_t position) const { return dense_.at(position); }
    int position(int x) const {
        check(x);
        return position_[x];
    }
    std::optional<int> sample(Random &random) const {
        if (empty())
            return std::nullopt;
        return dense_[random.uniform(size())];
    }
};
} // namespace cp
