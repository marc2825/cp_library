---
title: std::set/map/multiset の典型操作
documentation_of: //ordered_structure/ordered_associative.hpp
---

# std::set/map/multiset の典型操作

標準の順序付きコンテナをそのまま使い、前後検索と安全な取り出し・1 個だけの削除を補助します。set は重複なし、multiset は重複あり、map はキーから値への対応を管理します。

[補助関数の実装](../../ordered_structure/ordered_associative.hpp) / [操作できる図解](../../visualizations/ordered_structure/index.html#set) / [一覧](README.md)

## 標準コンテナで行う操作

n は現在の要素数、c は同値キーの個数です。

| 操作 | set / map | multiset | 注意点 |
|---|---|---|---|
| `insert(x)` / map の `try_emplace(key,value)` | O(log n)、既存キーを残す | O(log n)、重複も追加 | set/map の戻り値には挿入成功フラグがある |
| `find(key)` / `lower_bound(key)` / `upper_bound(key)` | O(log n) | O(log n) | メンバ関数を使う |
| `count(key)` | O(log n)、0 または 1 | O(log n+c) | 重複が多い場合は定数時間ではない |
| `erase(key)` | O(log n) | O(log n+c) | **同値キーを全削除** |
| `erase(iterator)` | ならし O(1) | ならし O(1) | end() は削除不可、次の iterator を返す |
| `size()` / `empty()` | O(1) | O(1) | 順位とは別 |
| `distance(begin,it)` / `next(begin,k)` | O(n) / O(k) | O(n) / O(k) | k 番目や順位は標準では O(log n) にならない |

既定の昇順では lower_bound(x) は最初の x 以上、upper_bound(x) は最初の x より大きい要素です。map ではキーで検索し、iterator の `first` がキー、`second` が対応値です。

## 補助 API

| API | 意味（昇順の場合） | 戻り値・計算量 |
|---|---|---|
| `cp::predecessor(c,x)` | 最大の x 未満 | iterator、O(log n) |
| `cp::predecessor(c,x,true)` | 最大の x 以下 | iterator、O(log n) |
| `cp::successor(c,x)` | 最小の x より大きい値 | iterator、O(log n) |
| `cp::successor(c,x,true)` | 最小の x 以上 | iterator、O(log n) |
| `cp::erase_one(c,x)` | 同値キー 1 個だけ削除 | bool、O(log n) |
| `cp::pop_first(c)` / `cp::pop_last(c)` | 順序の先頭／末尾をコピーして削除 | optional<value_type>、ならし O(1) ＋コピー |

検索は const にも対応し、該当なしは c.end() です。greater の場合はコンテナの**反復順序で前／後**になります。pop は空なら nullopt を返し、値のコピーが必要です。map の pop は `optional<pair<const Key,T>>` です。コピーできない mapped 型にも検索・erase_one は使えます。

## usage

```cpp
#include "ordered_structure/ordered_associative.hpp"
#include <map>
#include <set>
#include <string>
int main() {
    std::multiset<int> values{2, 2, 5};
    cp::erase_one(values, 2); // 2 は 1 個残る。values.erase(2) は全削除
    auto previous = cp::predecessor(values, 5);
    if (previous == values.end()) return 1;
    std::map<int, std::string> names;
    names.try_emplace(3, "first");
    names.try_emplace(3, "ignored"); // 既存値を保持
    names.insert_or_assign(5, "last");
    auto found = names.find(8); // 不在でも挿入しない
    return *previous != 2 || found != names.end() || names.size() != 2;
}
```

[動く使用例](../../examples/ordered_structure/standard_containers.cpp)。map の `m[key]` は不在キーを挿入します。参照だけなら find、存在を保証できる場合は at を使います。C++17 では contains は使わず、`find(x)!=end()` とします。

値域 `[l,r)` の削除は、比較器順で l<=r を確認して `c.erase(c.lower_bound(l),c.lower_bound(r))` とします。走査しながらの削除は `it=c.erase(it)` で進みます。挿入では既存 iterator は維持され、erase で無効になるのは削除した要素への iterator・参照です。

## 原理・注意点

補助関数はメンバの lower_bound/upper_bound/find を使い、begin/end の境界を確認してから iterator を戻します。標準コンテナの内部形状には依存しません。`std::lower_bound(c.begin(),c.end(),x)` は比較回数が少なくても iterator の移動に線形時間が必要なので、メンバ関数を使ってください。

キーを iterator 越しに変更してはいけません。頻度表として map を使うなら、0 個になったキーを erase するか、0 個のまま保持するかを統一します。map.size() は全個数の合計ではありません。頻度と順位も必要なら [圧縮＋BIT](compressed_order_statistics.md) を使えます。

## 使用する問題・検証

[Library Checker: Predecessor Problem](https://judge.yosupo.jp/problem/predecessor_problem) — [標準 set 版の検証プログラム](../../verify/ordered_structure/predecessor_std.test.cpp)。昇順・降順の set/multiset/map を線形走査と比較し、空集合、重複、const、透明な比較器の異種キー検索、取り出しを検証します。オンライン AC の記録ではありません。

一次資料: [C++17 草案 N4659・連想コンテナの要件](https://timsong-cpp.github.io/cppwp/n4659/associative.reqmts)、[map の要素アクセス](https://timsong-cpp.github.io/cppwp/n4659/map.access)。
