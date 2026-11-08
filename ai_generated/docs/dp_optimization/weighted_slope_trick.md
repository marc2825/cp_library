---
documentation_of: //dp_optimization/weighted_slope_trick.hpp
---

# 重み付きSlope Trick

[P-21の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#weighted_slope_trick) / [ヘッダ](../../dp_optimization/weighted_slope_trick.hpp)

`WeightedSlopeTrick` は初期関数f=0から、BigIntの座標・非負整数重みを持つ凸区分線形関数を更新します。`add_a_minus_x(a,w)`はw max(a-x,0)、`add_x_minus_a(a,w)`はw max(x-a,0)、`add_abs(a,w)`はw|x-a|を加えます。wの既定は1、負値は拒否します。`add_constant(c)`は任意符号の定数加算。

`prefix_min()`はmin_{y<=x} f(y)、`suffix_min()`はmin_{y>=x} f(y)。`shift(a,b)`はmin_{x-b<=y<=x-a} f(y)でa<=b、`shift(a)`はf(x-a)。`merge(other)`は関数和、自己merge可。`reflect()`はf(-x)。`minimum()`、最小区間の`argmin()`（左右のnulloptは無限端）、`evaluate(x)`、圧縮した`breakpoint_count()`を公開します。

左右mapにbreakpointと重みを保持し、交差する重みをまとめて移します。重み10^12を10^12回処理せず、必要なmap要素だけを消費します。全てBigIntなので座標差・値・巨大重みに固定幅の上限を設けません。BigInt演算自体の費用は桁数依存です。

breakpoint数K、更新で消費する要素数tならhinge追加O((t+1)log(K+1))、shift O(1)、評価O(K)、reflect O(K log(K+1))、prefix/suffixは削除要素数に比例、領域O(K)。mergeは相手をコピーし各hingeを追加する費用です。一般の関数減算は凸性を壊すため提供しません。

用途: 重み付き絶対偏差最小化、移動区間制約を持つDP、木の凸コスト合成。単位Slope Trickとの比較、整数点の全走査、自己merge、反転、10^30の重みと整数極値を検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/weighted_slope_trick.hpp"
int main() {
    cp::WeightedSlopeTrick f;
    f.add_abs(1,2); f.add_abs(5,3); f.shift(-1,2); f.reflect();
    auto [left,right] = f.argmin();
    std::cout << f.minimum() << ' ' << *left << ' ' << *right << ' ' << f.evaluate(0) << '\n';
}
```

出力:

```text
8 -7 -4 12
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
