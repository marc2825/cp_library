#pragma once
#include <algorithm>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cp {
// Algorithm X / Dancing Links。primaryは丁度1回、secondaryは高々1回。
// 各行はprimaryを1個以上含む。solve/enumerateは独立な作業コピーを使う。
class ExactCover {
    struct Node { int left, right, up, down, column, row; };
    int primary_, columns_, rows_ = 0;
    std::vector<Node> nodes_;
    std::vector<int> size_;
    void cover(int c) {
        nodes_[nodes_[c].left].right = nodes_[c].right;
        nodes_[nodes_[c].right].left = nodes_[c].left;
        for (int r = nodes_[c].down; r != c; r = nodes_[r].down)
            for (int j = nodes_[r].right; j != r; j = nodes_[j].right) {
                nodes_[nodes_[j].up].down = nodes_[j].down;
                nodes_[nodes_[j].down].up = nodes_[j].up;
                --size_[nodes_[j].column];
            }
    }
    void uncover(int c) {
        for (int r = nodes_[c].up; r != c; r = nodes_[r].up)
            for (int j = nodes_[r].left; j != r; j = nodes_[j].left) {
                ++size_[nodes_[j].column];
                nodes_[nodes_[j].up].down = j;
                nodes_[nodes_[j].down].up = j;
            }
        nodes_[nodes_[c].left].right = c;
        nodes_[nodes_[c].right].left = c;
    }
public:
    explicit ExactCover(int primary, int secondary = 0) : primary_(primary), columns_(0) {
        if (primary < 0 || secondary < 0 || primary > std::numeric_limits<int>::max() - secondary - 1)
            throw std::invalid_argument("invalid exact cover dimensions");
        columns_ = primary + secondary;
        size_.resize(static_cast<std::size_t>(columns_) + 1);
        for (int c = 0; c <= columns_; ++c)
            nodes_.push_back({c, c, c, c, c, -1});
        for (int c = 0; c <= primary; ++c) {
            nodes_[c].left = c ? c - 1 : primary;
            nodes_[c].right = c == primary ? 0 : c + 1;
        }
    }
    int add_row(std::vector<int> columns) {
        std::sort(columns.begin(), columns.end());
        if (columns.empty() || columns.front() < 0 || columns.back() >= columns_ ||
            columns.front() >= primary_ || std::adjacent_find(columns.begin(), columns.end()) != columns.end())
            throw std::invalid_argument("exact cover row: distinct columns and a primary required");
        if (columns.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) - nodes_.size() ||
            rows_ == std::numeric_limits<int>::max()) throw std::length_error("exact cover too large");
        nodes_.reserve(nodes_.size() + columns.size());
        const int first = static_cast<int>(nodes_.size()), count = static_cast<int>(columns.size());
        for (int k = 0; k < count; ++k) {
            const int c = columns[k] + 1, id = static_cast<int>(nodes_.size());
            nodes_.push_back({first + (k + count - 1) % count, first + (k + 1) % count,
                              nodes_[c].up, c, c, rows_});
            nodes_[nodes_[c].up].down = id; nodes_[c].up = id; ++size_[c];
        }
        return rows_++;
    }
    // callback(row IDs)がfalseなら終了。返値は通知した解数。limit=0なら探索なし。
    template<class Callback> std::size_t enumerate(Callback callback,
            std::size_t limit = std::numeric_limits<std::size_t>::max()) const {
        if (!limit) return 0;
        ExactCover work = *this;
        std::vector<int> selected;
        std::size_t count = 0;
        auto search = [&](auto&& self) -> bool {
            if (work.nodes_[0].right == 0) {
                ++count;
                return callback(std::as_const(selected)) && count < limit;
            }
            int c = work.nodes_[0].right;
            for (int j = work.nodes_[c].right; j != 0; j = work.nodes_[j].right)
                if (work.size_[j] < work.size_[c]) c = j;
            work.cover(c);
            for (int r = work.nodes_[c].down; r != c; r = work.nodes_[r].down) {
                selected.push_back(work.nodes_[r].row);
                for (int j = work.nodes_[r].right; j != r; j = work.nodes_[j].right)
                    work.cover(work.nodes_[j].column);
                if (!self(self)) return false; // 作業コピーを破棄するため復元不要。
                for (int j = work.nodes_[r].left; j != r; j = work.nodes_[j].left)
                    work.uncover(work.nodes_[j].column);
                selected.pop_back();
            }
            work.uncover(c);
            return true;
        };
        search(search);
        return count;
    }
    std::optional<std::vector<int>> solve() const {
        std::optional<std::vector<int>> result;
        enumerate([&](const auto& rows) { result = rows; return false; }, 1);
        return result;
    }
};
} // namespace cp
