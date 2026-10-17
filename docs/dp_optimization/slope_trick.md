---
title: Slope Trick
documentation_of: //dp_optimization/slope_trick.hpp
---

# Slope Trick

整数傾きの凸区分線形関数へhinge・絶対値を加え、最小値DPを更新します。

[実装](../../dp_optimization/slope_trick.hpp) / [操作できる図解](../../visualizations/math_a/index.html#slope_trick) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`SlopeTrick`はf=0から開始。`add_a_minus_x(a)`はmax(a-x,0)、`add_x_minus_a(a)`はmax(x-a,0)、`add_abs(a)`はabs(x-a)を加算。`add_constant(c)`、`minimum()`、`argmin()`（optionalの左右端、nulloptは非有界）を公開します。

`prefix_min()`はmin_{y<=x} f(y)、`suffix_min()`はmin_{y>=x} f(y)。`shift(a,b)`はmin_{x-b<=y<=x-a} f(y)、a<=bが必要。`shift(a)`はf(x-a)。`evaluate(x)`は非破壊の点評価、`merge(other)`はotherを変えずf+=other。自己mergeも可。

座標・オフセット・最小値・評価の中間値はlong long。計算時の超過はoverflow_error、逆向きshift区間はinvalid_argument。例外後の状態の強い保証はありません。傾きが単位ずつ変化する版で、巨大な重みを一回で追加するAPIは対象外です。

## 原理と計算量

左の最大heapと右の最小heapに折れ点を保持し、左の端点が右を超えた分だけ最小値を上げます。左右別のoffsetが区間最小化と平行移動を表します。

hinge/絶対値O(log(N+1))、定数加算・最小値・平行移動O(1)。prefix/suffixで捨てるheapの解放はO(N)。evaluateはコピーとpopでO(N log(N+1))時間/O(N)追加領域。M個のhingeを持つotherのmergeはO(M log(N+M+1))時間。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "dp_optimization/slope_trick.hpp"

int main() {
    cp::SlopeTrick f; f.add_abs(1); f.add_abs(5); f.shift(-1,2);
    auto [l,r]=f.argmin();
    std::cout << f.minimum() << " " << *l << " " << *r << "\n";
}
```

出力:

```text
4 0 7
```

[動く使用例](../../examples/dp_optimization/slope_trick.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [AtCoder ARC070 E - NarrowRectangles](https://atcoder.jp/contests/arc070/tasks/arc070_c)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [Nyaan Slope Trick](https://github.com/NyaanNyaan/library/blob/master/data-structure/slope-trick.hpp)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
