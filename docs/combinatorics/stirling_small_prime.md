---
title: 小素数法・巨大引数のStirling数
documentation_of: //combinatorics/stirling_small_prime.hpp
---

# 小素数法・巨大引数のStirling数（C-03）

[ヘッダ](../../combinatorics/stirling_small_prime.hpp) / [8項目の索引・検証](../combinatorics_extensions/README.md) / [操作できる図解](../../visualizations/combinatorics_extensions/index.html#stirling_small_prime)

## 使用例

```cpp
#include <iostream>
#include "combinatorics/stirling_small_prime.hpp"
int main() {
    cp::StirlingSmallPrime s(5);
    std::cout << s.first(1000000000003ULL, 3) << ' ' << s.second(1000000000003ULL, 3) << '\n';
}
```

出力:

```text
0 1
```

## API・条件

`StirlingSmallPrime(p,max_cells=4000000)`。`first(uint64 n,uint64 k,signed_values=false)` は第1種（既定は符号なしcycle数）、`second(n,k)` は第2種（集合分割数）。`modulus()` はp。k>nは0、(0,0)は1。

pは素数、2p²≤max_cellsを構築前に検査します。素数以外は `invalid_argument`、表予算超過は `length_error`。通常の `stirling_first/second` の三角表とは別APIです。

## 仕組み

第1種はn=qp+rとして、標数pでの上昇階乗

`x^(overline n) = (x^p−x)^q * x^(overline r)`

を利用します。後半の次数がp−1以下なので、係数kに寄与する項は高々2個です。二項係数をLucasで求め、符号付き版は最後に(−1)^(n−k)を掛けます。

第2種はk=qp+r、t=n−k、d=p−1とします。r>0なら `h_r(t)=[x^t] product_{i=1}^r (1−ix)^−1` は周期d。u=t/d、v=t%dとして `S(n,k)=h_r(v) C(q+u,u)`。r=0、k>0ならdがtを割る時のみ `C(q+t/d−1,t/d)` です。周期表hと小さい上昇階乗を前計算します。

## 計算量

前計算O(p²)時間・空間。問い合わせO(log_p(n+1))時間、追加O(1)空間。uint64全域でnに比例した処理をしません。

## 使用する問題

巨大な集合の分割数・巨大な順列のcycle数を小さい素数で評価し、必要ならCRT等へ渡す用途。

## 検証

p=2を含む8素数、n≤250の全kを従来三角表と比較。巨大uint64引数で両種の漸化式、n=k、k=0/1、符号を確認。 `python3 verify/run_combinatorics_extensions.py` が単独・重複include、全カテゴリ混在2翻訳単位、説明と同一の使用例、Debug/Releaseを確認します。図解は同じ使用例のC++出力を比較し、別のモデル検証も行います。オンラインACは未確認です。

[一次資料・導出と設計判断](../combinatorics_extensions/references.md) / [カテゴリ索引](README.md)
