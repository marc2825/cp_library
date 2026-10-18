---
title: Linear Matroid Parity
documentation_of: //combinatorial_optimization/linear_matroid_parity.hpp
---

# Linear Matroid Parity

ベクトル対を単位に選び、選択したすべての列が線形独立となる対の最大個数を求めます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#linear_matroid_parity) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`LinearMatroidPairs=vector<pair<vector<int>,vector<int>>>`。`linear_matroid_parity_exact<Mod>(pairs)`、`linear_matroid_parity<Mod>(pairs,trials=4,seed=20261018)`→pair添字列。Modは素数。

全ベクトルの次元は同じ。厳密版は小規模向け。代数版はΣxᵢ(uᵢvᵢᵀ−vᵢuᵢᵀ)のrankと削除テストを使うMonte Carlo法で、最大性に確率的な失敗があります。返す集合の独立性は必ず検査し、無限再試行をしません。小さい体での成功確率を保証せず、厳密性が必要ならexactを使います。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../combinatorial_optimization/linear_matroid_parity.hpp"

int main() {
    cp::LinearMatroidPairs pairs{{{1, 0, 0, 0}, {0, 1, 0, 0}}, {{0, 0, 1, 0}, {0, 0, 0, 1}}};
    std::cout << cp::linear_matroid_parity(pairs).size() << "\n";
}
```

出力:

```text
2
```

[コンパイルして動く使用例](../../examples/combinatorial_optimization/linear_matroid_parity_b.cpp)。

## 計算量

厳密版は最悪O(2^m poly(d,m))。代数版はO(trials*(m d³+m d²)+m*poly(d,m))の保守的上界、行列と入力の保存O(d²+md)。最初の貪欲実行可能集合の独立性判定を含みます。

## 使用する問題・検証

線形表現されたマトロイドの対選択、[yukicoder 1773](https://yukicoder.me/problems/no/1773)の関連する線形パリティ構造。

法3の全subset rankと厳密版を比較。大きい素数法では代数版と厳密版の最大サイズ、小さい法では返り値の実行可能性を検証。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
