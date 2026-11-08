---
title: 座標圧縮
documentation_of: //ordered_structure/coordinate_compression.hpp
---

# 座標圧縮

大きい整数・負数・文字列等を、順序を保った添字 0..M-1 に変換します。元の値を保存し、添字から元の値にも戻せます。

[実装](../../ordered_structure/coordinate_compression.hpp) / [値と添字の図解](../../visualizations/ordered_structure/index.html#compression) / [一覧](README.md)

## API・条件

`cp::CoordinateCompression<T,Compare=std::less<T>>`。比較器は厳密弱順序で、値は sort/unique に必要なコピー・移動・代入が可能な型とします。T のデフォルトコンストラクタ、==、整数への変換は不要です。

| API | 意味 | 計算量 |
|---|---|---|
| `CoordinateCompression(values={},compare={})` | ソートして同値を一つにまとめる | O(N log N) |
| `size()` / `empty()` | 異なる座標数 M／空判定 | O(1) |
| `index(x)` | 登録済みの同値座標の添字。未登録なら nullopt | O(log M) |
| `contains(x)` | 登録済みか | O(log M) |
| `lower_bound(x)` / `upper_bound(x)` | 最初の x 以上／x より大きい座標の位置 | O(log M) |
| `value(i)` | 添字 i の元の値への const 参照 | O(1) |
| `values()` | 全座標への const vector 参照 | O(1) |

lower/upper_bound は未登録の閾値にも使え、該当なしは M を返します。これは index(x) の nullopt とは区別します。value(i) は `0<=i<M` が前提で、デバッグ時に assert で検査します。

構築中のメモリ O(N)、構築後の論理座標数 O(M)。vector は元の容量を保持する場合があるため、確保メモリは O(N) のことがあります。入力長 N は 1 億以下です。構築後の座標追加はありません。

## usage

```cpp
#include "ordered_structure/coordinate_compression.hpp"
#include <vector>
int main() {
    std::vector<long long> a{100, -5, 100, 20};
    cp::CoordinateCompression<long long> coords(a); // {-5,20,100}
    auto index = coords.index(100);                // 2
    auto absent = coords.index(10);                // nullopt
    int cut = coords.lower_bound(10);              // 1
    return !index || *index != 2 || absent || cut != 1 || coords.value(1) != 20;
}
```

[動く使用例](../../examples/ordered_structure/compression.cpp)。複数の入力配列を同じ基準で圧縮する場合は、それらの値を先に一つの vector に集めて構築します。

## 原理・注意点

ソート後、比較器で同値な隣接要素を unique でまとめます。同値クラスの代表値はソート結果によるため、独自比較器で異なる値を同値にする場合、最初の入力が残る保証はありません。

圧縮は順序を保存しますが、距離を保存しません。元の値 20 と 100 の距離 80 は、添字差 1 にはなりません。元の配列 a は変更しません。旧 `misc/zaatu.hpp` は入力を in-place に書き換える既存 API のため、そのまま残しています。

## 使用する問題・検証

[Library Checker: Ordered Set](https://judge.yosupo.jp/problem/ordered_set) と [AtCoder ARC033 C](https://atcoder.jp/contests/arc033/tasks/arc033_3) の [順位管理実装](compressed_order_statistics.md) で、挿入候補の圧縮に使っています。重複・負数・端の整数・文字列・昇降順・独自同値判定・未登録閾値・空配列を検証します。オンライン AC の記録ではありません。

設計と参考資料は [references.md](references.md) に記載しています。
