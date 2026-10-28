---
title: 固定1〜3次元の厳密LP
documentation_of: //combinatorial_optimization/low_dimensional_lp.hpp
---

# O-02 固定1〜3次元の厳密LP

[カテゴリ](README.md) / [実装](../../combinatorial_optimization/low_dimensional_lp.hpp) / [操作できる図解](../../visualizations/optimization_extensions/index.html#low_dimensional_lp) / [一次資料](references.md)

仮想的な[-M,M]^dの頂点をu+Mvとして解き、Mの係数を優先して比較する。

## API・条件

`low_dimensional_lp<D>` はD=1,2,3をコンパイル時に指定します。各行は `array<BigRational,D>`、変数は符号自由です。A·x≤bでc·xを最大化し、浮動のepsilonを使いません。

有限の箱で切ると、箱の外にある解や非有界性を誤判定します。そこで制約へ−M≤xⱼ≤Mを加え、Mを全ての有理定数より大きい記号として保持します。D本の独立な境界を選び、既存Gauss–Jordanで頂点をu+Mvの形で解きます。不等式と目的はMの係数から比較します。

目的のM係数が正の実行可能頂点を見つければ、そのvが非有界方向です。全候補を調べて実行可能頂点がなければinfeasible、有限の最大値を得たらoptimal。点・線・平面へ退化した領域、全空間、目的0もこの規則に含まれます。解点には原制約を満たす十分大きい有理Mを計算して代入します。

`LowDimensionalLPResult` はstatus/has_primal/value/primal/ray/basesを使用します。共通結果型を継承しますがdualは返さず、pivotsは0です。max_basesの既定は100000。候補数は特異な組合せも含み、予算終了ならiteration_limitと見つかった実行可能点を返します。その点の最適性は未確定です。

これは小さい次元・少数制約向けの厳密な基底列挙です。期待線形時間のSeidel法ではありません。制約が多い場合は[Simplex](linear_program.md)も選択肢です。

## 計算量

m制約・固定dでC(m+2d,d)候補、各O(d³+md)有理演算、O(md+d²)領域。max_basesで打切り。

## 使用例

```cpp
#include "combinatorial_optimization/low_dimensional_lp.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto r = low_dimensional_lp<2>({{{1, 1}}, {{-1, 0}}, {{0, -1}}}, {4, 0, 0}, {{2, 1}});
    std::cout << lp_status_name(r.status) << " " << r.value << "\n";
}
```

出力:

```text
optimal 8/1
```

## 使用する問題・検証

2D/3Dの半空間制約、退化した領域上の線形目的。 オンラインACは未確認です。

[単体・全探索比較](../../verify/optimization_extensions/unit_test.cpp)、[独立Python比較](../../verify/run_optimization_extensions.py)、[使用例と参照値](../../verify/optimization_extensions/manifest.json)で検証します。検証コマンドと図解との相違は[カテゴリREADME](README.md)を参照してください。
