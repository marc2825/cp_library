---
title: Lagrange Inversion
documentation_of: //combinatorics/lagrange_inversion.hpp
---

# Lagrange Inversion

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#lagrange_inversion) / [実装](../../combinatorics/lagrange_inversion.hpp) / [一次資料と設計](../b_12_15/references.md)

g=xφ(g)で暗黙定義される形式的冪級数について、F(g)の指定係数を直接求めます。

## usage

```cpp
#include <iostream>
#include "combinatorics/lagrange_inversion.hpp"
int main() {
    std::cout << cp::lagrange_inversion_coefficient(std::vector<int>{1,2,1},std::vector<int>{0,1},4) << '\n';
}
```

出力:

```text
14
```

## API・使用条件

`lagrange_inversion_coefficient<Mod,Root>(phi,function,n)`→int。係数は昇冪vector<int>、素数Mod、0<=n<Mod。n=0はF(0)。φ(0)=0も可。既存FPSの精度上限に従う。

## 仕組み・境界条件

[x^n]F(g)=[t^(n−1)]F'(t)φ(t)^n/n。既存fps_powで必要なprefixだけを求めます。関数gの全係数や合成逆関数そのものを返すAPIではありません。0系列・定数Fも扱います。

## 計算量

O(N log N)時間、O(N)領域（既存FPS冪）。入力のnより先の係数は不要。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

暗黙母関数、根付き木・Catalan型の係数抽出。 [動く小問題](../../examples/combinatorics/lagrange_inversion_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
