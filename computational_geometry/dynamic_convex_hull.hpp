#pragma once
#include "detail/robust_hull.hpp"
#include <list>
#include <map>
namespace cp::geometry::exact {
// Fully dynamic multiset with block hulls. B is explicit; updates O(n/B + B log B).
class DynamicConvexHull {
    struct Block {
        std::map<Point, std::size_t> points;
        std::vector<Point> hull, lower, upper;
    };
    std::list<Block> blocks_;
    std::size_t block_size_, size_ = 0;
    mutable bool dirty_ = true;
    mutable std::vector<Point> cached_;
    static void rebuild(Block &block) {
        std::vector<Point> points;
        for (auto entry : block.points)
            points.push_back(entry.first);
        block.hull = robust::convex_hull(std::move(points));
        block.lower.clear();
        block.upper.clear();
        if (block.hull.empty())
            return;
        auto right = std::max_element(block.hull.begin(), block.hull.end());
        block.lower.assign(block.hull.begin(), right + 1);
        block.upper.push_back(block.hull[0]);
        for (auto it = block.hull.end(); it != right + 1;)
            block.upper.push_back(*--it);
        if (right != block.hull.begin())
            block.upper.push_back(*right);
    }
    auto find_block(Point p) {
        auto it = blocks_.begin();
        while (std::next(it) != blocks_.end() && it->points.rbegin()->first < p)
            ++it;
        return it;
    }
    static BigInt product(Point a, Point b) { return BigInt(a.x) * b.x + BigInt(a.y) * b.y; }

  public:
    explicit DynamicConvexHull(std::size_t block_size = 128) : block_size_(block_size) {
        if (!block_size || block_size > std::numeric_limits<std::size_t>::max() / 2)
            throw std::invalid_argument("invalid hull block size");
    }
    std::size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    void insert(Point p) {
        if (blocks_.empty())
            blocks_.emplace_back();
        auto it = find_block(p);
        ++size_;
        if (++it->points[p] > 1)
            return;
        dirty_ = true;
        if (it->points.size() > 2 * block_size_) {
            auto next = blocks_.emplace(std::next(it));
            auto middle = it->points.begin();
            std::advance(middle, it->points.size() / 2);
            next->points.insert(middle, it->points.end());
            it->points.erase(middle, it->points.end());
            rebuild(*next);
        }
        rebuild(*it);
    }
    bool erase(Point p) {
        if (blocks_.empty())
            return false;
        auto it = find_block(p);
        auto entry = it->points.find(p);
        if (entry == it->points.end())
            return false;
        --size_;
        if (--entry->second)
            return true;
        it->points.erase(entry);
        dirty_ = true;
        if (it->points.empty()) {
            blocks_.erase(it);
            return true;
        }
        // Every block (unless it is the only one) retains at least B/2 points.
        // Merge or redistribute an underfull block even when its neighbor is full.
        if (blocks_.size() > 1 && it->points.size() < block_size_ / 2) {
            auto next = std::next(it);
            if (next == blocks_.end()) {
                next = it;
                --it;
            }
            it->points.insert(next->points.begin(), next->points.end());
            if (it->points.size() <= 2 * block_size_) {
                blocks_.erase(next);
            } else {
                next->points.clear();
                auto middle = it->points.begin();
                std::advance(middle, it->points.size() / 2);
                next->points.insert(middle, it->points.end());
                it->points.erase(middle, it->points.end());
                rebuild(*next);
            }
        }
        rebuild(*it);
        return true;
    }
    std::optional<Point> support(Point direction) const {
        std::optional<Point> best;
        for (const auto &block : blocks_) {
            const auto &chain = direction.y >= 0 ? block.upper : block.lower;
            std::size_t l = 0, r = chain.size() - 1;
            while (l < r) {
                const auto m = l + (r - l) / 2;
                if (product(chain[m], direction) <= product(chain[m + 1], direction))
                    l = m + 1;
                else
                    r = m;
            }
            const auto p = chain[l];
            if (!best || product(*best, direction) < product(p, direction))
                best = p;
        }
        return best;
    }
    const std::vector<Point> &hull() const {
        if (dirty_) {
            std::vector<Point> points;
            for (const auto &block : blocks_)
                points.insert(points.end(), block.hull.begin(), block.hull.end());
            cached_ = robust::convex_hull(std::move(points));
            dirty_ = false;
        }
        return cached_;
    }
    Location contains(Point p) const {
        return geometry::detail::convex_contains(hull(), p, robust::IntegerKernel{});
    }
};
} // namespace cp::geometry::exact
