---
title: 適応Simpson積分
documentation_of: //numerical_analysis/integration.hpp
---

# Y-02 適応Simpson積分

[カテゴリ](README.md) / [実装](../../numerical_analysis/integration.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#adaptive_simpson) / [参考資料](references.md)

粗いSimpsonと二分したSimpsonの差/15を推定誤差にし、全体の誤差予算を超える間、最大誤差の区間を細分化する。

## API・使用条件

`adaptive_simpson(f,a,b,IntegrationOptions)` → value / error_estimate / status / evaluations / panels。

有限区間の十分滑らかな関数が対象。特異点・不連続・鋭い未観測の山・高周波では推定誤差は保証にならない。事前に区間分割や変数変換を行う。max_depth/max_evaluationsと点の丸め停滞を区別。コールバック例外は伝播。

`adaptive_simpson(f,a,b,IntegrationOptions)`。1区間でのSimpson値Sと、左右2区間の合計S₂から δ=(S₂−S)/15、補正値S₂+δ、推定誤差|δ|を得ます。4階の主要誤差が二分で1/16になる仮定から15が現れます。最初に5点を評価し、推定誤差が最大の区間を優先して分割します。1回の分割は4点を追加評価します。

全区間の推定誤差の和が `absolute+relative*abs(value)` 以下で `converged`。`max_depth=0` でも最初の5点は評価でき、さらに分割が必要なら `max_depth` です。評価予算・丸めによる分割不能は別status。途中終了時は最後に完成した区間分割の値と誤差を返し、初期5点すら完成していなければpanels=0、誤差∞です。

誤差は厳密な上界ではありません。細いピークや高周波が全評価点をすり抜けると過小評価します。不連続点が既知ならそこで分割し、特異点・無限区間は適切な変数変換や別手法を使ってください。端点を評価するため、端点で非有限な被積分関数は `non_finite` です。相殺のある積分では正の絶対許容誤差を指定してください。

例えば f(x)=2−3x²+5x⁴−4x⁶ を[−1,1]で評価すると、最初の推定誤差は0、近似値は8/3ですが、真の積分は20/7です。この例は0で区間を分けると正しい値へ収束します。これは滑らかでも起きる推定の限界であり、回帰テストで明示しています。

## 計算量

P葉区間でO(PF+P log P)時間、O(P)領域。初期5評価、1回の細分化に4評価。

## 使用例

```cpp
#include "numerical_analysis/integration.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    auto result = adaptive_simpson([](long double x) { return std::exp(x); }, 0, 1);
    std::cout << result.value << "\n";
}
```

出力:

```text
1.71828
```

## 使用する問題・検証

区間ごとに変化の速さが異なる関数の面積・期待値。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
