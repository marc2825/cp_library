#pragma once
#include <cassert>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

// Candidate stack for nearest previous/next better elements; less<T> means smaller.
// strict=true: < (discard equals); false: <= (keep equals). Missing index is -1.
// Docs: docs/static_query/monotonic_stack.md
namespace cp {

template <class T, class Compare = std::less<T>> class MonotonicStack {
  public:
    struct Entry { int index; T value; };
    explicit MonotonicStack(bool strict = true, Compare compare = Compare{})
        : strict_(strict), compare_(std::move(compare)) {}
    // Returns the nearest qualifying predecessor BEFORE inserting this element.
    int push(T value) {
        assert(next_ < std::numeric_limits<int>::max());
        while (!data_.empty() &&
               (strict_ ? !compare_(data_.back().value, value)
                        : compare_(value, data_.back().value))) data_.pop_back();
        int previous = data_.empty() ? -1 : data_.back().index;
        data_.push_back({next_++, std::move(value)});
        return previous;
    }
    int size() const { return int(data_.size()); } // Number of retained candidates.
    int processed_count() const { return next_; }
    bool empty() const { return data_.empty(); }
    const Entry& top() const { assert(!empty()); return data_.back(); }
    const std::vector<Entry>& candidates() const { return data_; }
    void clear() { data_.clear(); next_ = 0; }

  private:
    bool strict_;
    Compare compare_;
    int next_ = 0;
    std::vector<Entry> data_;
};

template <class T, class Compare = std::less<T>>
std::vector<int> previous_better_indices(const std::vector<T>& a,
                                       bool strict = true, Compare compare = Compare{}) {
    assert(a.size() <= static_cast<std::size_t>(std::numeric_limits<int>::max()));
    MonotonicStack<T, Compare> stack(strict, std::move(compare));
    std::vector<int> answer;
    answer.reserve(a.size());
    for (const T& x : a) answer.push_back(stack.push(x));
    return answer;
}

template <class T, class Compare = std::less<T>>
std::vector<int> next_better_indices(const std::vector<T>& a,
                                   bool strict = true, Compare compare = Compare{}) {
    assert(a.size() <= static_cast<std::size_t>(std::numeric_limits<int>::max()));
    int n = int(a.size());
    std::vector<int> answer(n, -1), stack;
    for (int i = n - 1; i >= 0; --i) {
        while (!stack.empty() &&
               (strict ? !compare(a[stack.back()], a[i])
                       : compare(a[i], a[stack.back()]))) stack.pop_back();
        if (!stack.empty()) answer[i] = stack.back();
        stack.push_back(i);
    }
    return answer;
}

}  // namespace cp
