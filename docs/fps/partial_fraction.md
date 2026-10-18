---
title: Partial Fraction Decomposition
documentation_of: //fps/partial_fraction.hpp
---

# Partial Fraction Decomposition

分母の全根と重複度が既知の有理式を、多項式部と各極の主部へ分解します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#partial_fraction) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`partial_fraction_decomposition<Mod,Root>(numerator,roots,multiplicities)`→`{polynomial_part,coefficients}`。coefficients[i][j-1]は1/(x-roots[i])^jの係数。

根は法で相異なる、重複度は正、D+1≤L、D≤Mod、有効分子長≤L。Q=Π(x-root)^mはmonic。各極でR(x+root)/(Q/(x-root)^m)(x+root)の低次m項を逆順に主部へ配置します。極が空ならQ=1、多項式部のみ。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../fps/partial_fraction.hpp"

int main() {
    auto parts = cp::partial_fraction_decomposition(std::vector<int>{1}, {1}, {2});
    std::cout << parts.coefficients[0][0] << " " << parts.coefficients[0][1] << "\n";
}
```

出力:

```text
0 1
```

[コンパイルして動く例](../../examples/fps/partial_fraction_b.cpp)。

## 計算量

分母次数D・分子長A・極数rとして、分母構築O(D² log(D+1))の保守的上界、極ごとに除算・Taylor Shift・局所逆数を実行。O(D² log(D+1)+r M(max(D,A)))時間の保守的上界、保存O(D+A)。

## 使用する問題・検証範囲

有理母関数の極ごとの分解、重根から生じる多項式重み付き等比列。

複数の重根と多項式部を持つ式を共通分母へ戻し、元の分子と比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
