/*
★ Name        : Cartesian Tree
 ■ Snipet      : No

 ■ Arguments   : values, Compare, tie break
 ■ Return      : root, parent, left, right
 ■ Complexity  : O(N) time / O(N) memory

 ■ Description :
   中順巡回 (inorder traversal) した頂点列が 0, 1, ..., N-1 となり、Compare に関して heap となる二分木。
   std::less<T> (既定) で最小 Cartesian Tree、std::greater<T> で最大 Cartesian Tree を構築する。
   重複値は tie break により、各区間の同順位値のうち最左/最右の index を根とする。
   ここで同順位とは !Compare(a,b) && !Compare(b,a) を満たすことで、operator== は不要。
   頂点は 0-indexed。根の parent と存在しない子は -1。空列の root も -1。

 ■ Usage       :
   CartesianTree<int> min_tree(a);
   CartesianTree<int, std::greater<int>> max_tree(a);
   CartesianTree<int> rightmost_tree(a, CartesianTreeTieBreak::rightmost);
   // Library Checker の出力: min_tree.parent_or_self(i)

 ■ Verify      : local exhaustive/randomized differential test (ASan/UBSan)
 ■ References  : https://drken1215.hatenablog.com/entry/2023/10/19/025800
                 https://judge.yosupo.jp/problem/cartesian_tree

 ■ TODO        :
*/

#pragma once

#include <cassert>
#include <cstddef>
#include <functional>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

enum class CartesianTreeTieBreak {
    leftmost,
    rightmost,
};

/// Cartesian Tree を monotonic stack で構築する。
// Compare(a, b) == true のとき a の優先度が b より高いものとする。
// Compare は strict weak ordering を満たす必要がある。
// 比較器を省略する場合、Compare{} が有効な比較器である必要がある。
// 値自体は保持せず、頂点番号と木構造だけを保持する。
template <class T, class Compare = std::less<T>>
struct CartesianTree {
    static constexpr int none = -1;

    int root = none;
    std::vector<int> parent;
    std::vector<int> left;
    std::vector<int> right;

    CartesianTree() = default;

    /// 既定の比較器で構築する。
    explicit CartesianTree(
        const std::vector<T>& values,
        CartesianTreeTieBreak tie_break = CartesianTreeTieBreak::leftmost)
        : tie_break_(tie_break) {
        build(values);
    }

    /// 状態を持つ比較器も使えるコンストラクタ。
    CartesianTree(
        const std::vector<T>& values,
        Compare compare,
        CartesianTreeTieBreak tie_break = CartesianTreeTieBreak::leftmost)
        : compare_(std::move(compare)), tie_break_(tie_break) {
        build(values);
    }

    /// 現在の Compare と tie break を使って再構築する。
    // 空列に対しては root == none で、各 vector も空になる。
    void build(const std::vector<T>& values) {
        if (values.size() >
            static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            throw std::length_error(
                "CartesianTree supports at most INT_MAX vertices");
        }
        const int n = static_cast<int>(values.size());

        // T == int のとき values が自分の構造配列を参照していても、
        // 初期化で入力を壊さないように先に退避する。
        const std::vector<T>* source = &values;
        std::vector<int> values_copy;
        if constexpr (std::is_same_v<T, int>) {
            if (&values == &parent || &values == &left || &values == &right) {
                values_copy = values;
                source = &values_copy;
            }
        }
        const std::vector<T>& a = *source;

        root = none;
        parent.assign(n, none);
        left.assign(n, none);
        right.assign(n, none);

        // stack は処理済みの木の root から右端までの経路を保持する。
        std::vector<int> stack;
        stack.reserve(n);
        const bool pop_equal =
            tie_break_ == CartesianTreeTieBreak::rightmost;

        for (int i = 0; i < n; ++i) {
            int last = none;
            while (!stack.empty()) {
                const int j = stack.back();
                bool i_has_higher_priority = compare_(a[i], a[j]);
                if (!i_has_higher_priority && pop_equal) {
                    i_has_higher_priority = !compare_(a[j], a[i]);
                }
                if (!i_has_higher_priority) break;

                last = j;
                stack.pop_back();
            }

            if (!stack.empty()) {
                parent[i] = stack.back();
                right[stack.back()] = i;
            }
            if (last != none) {
                parent[last] = i;
                left[i] = last;
            }
            stack.push_back(i);
        }

        if (!stack.empty()) root = stack.front();
    }

    int size() const noexcept { return static_cast<int>(parent.size()); }
    bool empty() const noexcept { return parent.empty(); }

    /// Library Checker のように、根の親を根自身として取得する。
    int parent_or_self(int v) const {
        assert(0 <= v && v < size());
        return parent[v] == none ? v : parent[v];
    }

    CartesianTreeTieBreak tie_break() const noexcept { return tie_break_; }

  private:
    Compare compare_{};
    CartesianTreeTieBreak tie_break_ = CartesianTreeTieBreak::leftmost;
};

template <class T>
using MinCartesianTree = CartesianTree<T, std::less<T>>;

template <class T>
using MaxCartesianTree = CartesianTree<T, std::greater<T>>;
