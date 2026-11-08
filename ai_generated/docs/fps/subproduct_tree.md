---
title: Subproduct Tree
documentation_of: //fps/subproduct_tree.hpp
---

# Subproduct Tree

点集合の積木を保存し、同じ点での評価・補間を繰り返せます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#subproduct_tree) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`SubproductTree<Mod,Root>(points)`、`size()`、`product()`（const参照）、`evaluate(f)`、`interpolate(values)`。点数<L、評価多項式の有効長≤L、L=fps_max_size。

積はΠ(x-points[i])。評価は重複点可、補間は法で相異なる点が必要。空の積は{1}、空の補間・評価は空。既存polynomial_interpolationもこの公開型を使うよう統合しました。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../fps/subproduct_tree.hpp"

int main() {
    cp::SubproductTree<> tree({1, 2, 3});
    auto values = tree.evaluate({1, 0, 1});
    std::cout << values[2] << " " << tree.interpolate(values)[2] << "\n";
}
```

出力:

```text
10 1
```

[コンパイルして動く例](../../examples/fps/subproduct_tree_b.cpp)。

## 計算量

構築・補間O(M(m+1)log(m+1))、評価O(M(d)+M(m+1)log(m+1))。保存O(m log(m+1))。

## 使用する問題・検証範囲

[Multipoint Evaluation](https://judge.yosupo.jp/problem/multipoint_evaluation)、[Polynomial Interpolation](https://judge.yosupo.jp/problem/polynomial_interpolation)。

Horner評価、補間の往復、同じ木の再利用、重複点と空を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
