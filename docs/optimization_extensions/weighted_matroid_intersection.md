---
title: 重み付きマトロイド交差
documentation_of: //combinatorial_optimization/weighted_matroid_intersection.hpp
---

# O-20 重み付きマトロイド交差

[カテゴリ](README.md) / [実装](../../combinatorial_optimization/weighted_matroid_intersection.hpp) / [操作できる図解](../../visualizations/optimization_extensions/index.html#weighted_matroid_intersection) / [一次資料](references.md)

追加要素に−w、削除要素に+wの費用を置き、費用・辺数の順で最短の増加路を選ぶ。

## API・条件

weightsはlong long列、2つのoracleは既存 `matroid_intersection` と同じ独立性判定です。戻り値のselected/weightは全ての要素数を通じた最大重み共通独立集合。by_size[k]は要素数kに限定した最大重み解で、k=0の空集合から最大要素数まで連続します。

負重みを許すため、最大要素数の解が全体の最適解とは限りません。最大要素数を優先したい場合はby_size.back()を使います。全体の重みが同じなら要素数が大きい解を選びます。同じ要素数・重みでの辞書順最小は保証しません。

交換グラフの外側要素へ入る費用を−w、内側要素を外す費用を+wにします。各要素数で既に最良の解から、費用最小、同値なら辺数最少の増加路をBellman–Fordで求めます。辺数の同値規則を省略すると増加後の独立性を壊す場合があります。

追加ごとに両oracleで独立性を再確認します。マトロイド公理・決定性・要素順に依存しないことは入力契約であり、全公理の検査はしません。矛盾した増加や負閉路を検出した場合はlogic_error。費用と重みの128bit演算は既存のOptimizationWideを共用し、表示はoptimization_to_stringです。

## 計算量

nは要素数、rは最大共通独立集合の要素数、Fはoracle1回の費用です。各増加と最後の探索につきO(n²)oracle呼出しとO(n³)Bellman–Ford。集合構築込みO((r+1)(n+1)²(F+n+1))時間、結果込みO((n+1)²)領域。

## 使用例

```cpp
#include "combinatorial_optimization/weighted_matroid_intersection.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto independent = [](std::vector<int> groups) {
        return [groups](const std::vector<int> &s) {
            std::vector<int> count(2);
            for (int e : s)
                if (++count[groups[e]] > 1)
                    return false;
            return true;
        };
    };
    auto r = weighted_matroid_intersection(std::vector<long long>{8, 7, 6, 1}, independent({0, 0, 1, 1}),
                                           independent({0, 1, 0, 1}));
    std::cout << optimization_to_string(r.weight) << "\n";
    for (std::size_t i = 0; i < r.selected.size(); i++)
        std::cout << (i ? " " : "") << r.selected[i];
    std::cout << "\n";
}
```

出力:

```text
13
1 2
```

## 使用する問題・検証

色制約付き森林、容量付き二部割当、重みと要素数の両方を調べる問題。 オンラインACは未確認です。

[単体・全探索比較](../../verify/optimization_extensions/unit_test.cpp)、[独立Python比較](../../verify/run_optimization_extensions.py)、[使用例と参照値](../../verify/optimization_extensions/manifest.json)で検証します。検証コマンドと図解との相違は[カテゴリREADME](README.md)を参照してください。
