---
title: Black-Box Linear Algebra
documentation_of: //linear_algebra/black_box.hpp
---

# Black-Box Linear Algebra

行列を保存せず、固定の線形作用apply(x)=A*xだけを受け取ってKrylov列・冪作用・候補解を求めます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#black_box) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`black_box_krylov_sequence<Mod>(n,apply,left,right,count)`、`black_box_power_apply<Mod>(n,apply,v,uint64 exponent,annihilator)`、`black_box_solve<Mod>(n,apply,b,annihilator)`。多項式は昇冪monic正次数。

全入力ベクトルとcallbackの出力次元を検査。callbackは同じ線形写像を表し、入力を変更しないこと。powerはm(A)v=0を検証し、違えばdomain_error。solveはm[0]≠0から候補を作り、必ずA*x=bを確認。nulloptは解なしの証明ではありません。solveは素数法、Krylov/powerは合成数法も使えます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_algebra/black_box.hpp"

int main() {
    auto apply = [](const std::vector<int>& x) { return x; };
    auto y = cp::black_box_power_apply(2, apply, std::vector<int>{2, 3}, 100, std::vector<int>{-1, 1});
    std::cout << y[0] << " " << y[1] << "\n";
}
```

出力:

```text
2 3
```

[コンパイルして動く例](../../examples/linear_algebra/black_box_b.cpp)。

## 計算量

matvec費用T、次数dとしてKrylovはO(count(T+n))、solveはO(d(T+n)+log Mod)、powerはO(d(T+n)+d²log(exponent+1))時間。保存O(n+d+count)、powerはcount不要。

## 使用する問題・検証範囲

巨大な線形遷移の適用、既知の消去多項式からの求解。

直接の行列冪作用、残差、誤った多項式・callback形状を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
