---
title: 黄金分割探索・整数フィボナッチ探索
documentation_of: //basic_utility/unimodal_search.hpp
---

# 黄金分割探索・整数フィボナッチ探索（U-06）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#unimodal_search)

黄金比またはフィボナッチ数の比率で内点を置き、比較後も片方の観測値を再利用します。整数版は区間をフィボナッチ長へ仮想的に拡張して、丸めに依存しない点配置を使います。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/unimodal_search.hpp"
#include <cmath>
#include <iomanip>
int main() {
    auto integer = cp::fibonacci_search(-10, 11, [](std::int64_t x) { return std::abs(x - 3); });
    auto real = cp::golden_section_search(-10, 10, [](long double x) { return std::abs(x - 3); });
    std::cout << integer.position << ' ' << integer.value << '\n';
    std::cout << std::fixed << std::setprecision(6) << real.position << '\n';
}
```

出力:

```text
3 0
3.000000
```

## API・使用条件

両APIは最小化です。関数が「厳密に減少→最小値の平坦区間→厳密に増加」の形（片側が空でも可）で、同じ入力では同じ値を返すことが前提です。凸関数はこの条件を満たします。最小値以外の平坦部を含む弱い単峰性だけでは正解を保証しません。最大化は比較を逆にする目的関数などへ変換します。

`golden_section_search(left,right,f,iterations=80)` は実数の**閉区間**。`GoldenSectionResult{position,value,left,right,evaluations}` を返します。position/valueは観測済み最良点、left/rightは残った探索区間です。端点も評価します。丸めで内点が重なれば終了。有限の端点・区間幅が必要で、逆順・負の反復数は `invalid_argument`、非有限関数値は `domain_error`。位置精度と関数値精度は別で、浮動小数点比較による誤差があります。許容誤差による自動終了ではありません。

`fibonacci_search(int64_t left,right,f)` は整数の**半開区間**。空・逆順は例外。`FibonacciSearchResult<T>{position,value,evaluations}` は厳密な最小値とその最左位置です。目的値型Tはコピー可能で順序比較が可能な型。浮動小数点の非有限値は拒否します。入力区間外の関数評価は行いません。区間幅・フィボナッチ数・中点の算術は128bitで、64bit全域に近い区間にも対応します。

## 計算量

黄金分割はI回の縮小で最大I+4回の評価（同一点区間は1回）、O(1)記憶。実数算術を仮定すれば幅は黄金比の逆数のI乗に比例して減少します。

整数探索はN=right−leftに対し F_k≥N+1 を満たす最小k≥3を選び、N=1で1回、それ以外は高々k−2回の実関数評価。O(log(N+1))時間・記憶。各縮小で1点の評価を再利用し、範囲外の仮想点は+∞として比較するため評価しません。

## 使用する問題・検証

評価に時間のかかる単峰な費用関数の最小化。左右でランダムな傾きを持つ列・平坦な最小区間・境界の最小値を走査結果と比較し、評価の重複・回数・区間外呼出しも検査します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
