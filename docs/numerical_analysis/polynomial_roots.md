---
title: 多項式の実根列挙・Sturm分離
documentation_of: //numerical_analysis/polynomial_roots.hpp
---

# Y-04 多項式の実根列挙・Sturm分離

[カテゴリ](README.md) / [実装](../../numerical_analysis/polynomial_roots.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#polynomial_roots) / [参考資料](references.md)

fとf′のGCDで重複因子を除き、Sturm列の符号変化数の差を保ちながら二分する。

## API・使用条件

`real_polynomial_roots(vector<long double>, options)`、`isolate_real_roots(vector<BigRational>,options)`。係数は昇べき順。

実際の二進係数を厳密に有理数へ変換。異なる実根を一度ずつ数え、重複度は返さない。全0はall_real。根区間は(a,b]、完全一致は一点。予算超過でも未分離clusterのcountを含め全根を保持。valueは近似値で、証明は有理区間。

係数は定数項から昇べき順です。末尾の0は除去します。`real_polynomial_roots` は受け取ったlong doubleの実際の二進値を厳密な有理数に変換し、`isolate_real_roots` と同じ実体を使います。10進入力の理想値に対する証明ではありません。有限体の根計算とは別APIで、Boostヘッダが必要です。

f/gcd(f,f′)で重根を除いた多項式のSturm列を作り、0を除いた符号変化数V(a)−V(b)で区間(a,b]の相異なる実根数を厳密に数えます。Cauchy上界の内側に全根を囲み、区間を二分します。1根かつ幅≤absolute+relative*max(|a|,|b|)なら分離完了。右端がその根に一致すればlower=upperの一点区間です。近い2根を許容誤差で同じ根にまとめません。

結果は `distinct_roots`、`subdivisions` と昇順の `intervals`。区間ごとにlower/upper/count/valueがあります。valueは中点の近似であり、count>1のclusterでは特定の根を近似しません。`max_subdivisions` を使い切ると `max_iterations` になり、全未処理区間もcount付きで返します。count=1でも予算終了時には幅が許容誤差を満たさない場合があります。近似がoverflowしたときは `non_finite` ですが、有理区間は保持します。全0はall_real、非零定数は空列です。

次数上限超過はlength_error。厳密演算のbit数は増大し得るので、大次数の高速数値固有値法の代用にはしません。特に小さい許容誤差と極端な係数は、次数が低くても重くなります。

## 計算量

次数d、細分化SとしてO((S+1)d²)回の有理演算、O(d²)領域。bit成長は別。既定max_degree=64、max_subdivisions=100000は厳密な時間・メモリ予算ではない。

## 使用例

```cpp
#include "numerical_analysis/polynomial_roots.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    const auto result = real_polynomial_roots({0, -2, 1, 1});
    for (std::size_t i = 0; i < result.intervals.size(); ++i)
        std::cout << (i ? " " : "") << result.intervals[i].value;
    std::cout << "\n";
}
```

出力:

```text
-2 0 1
```

## 使用する問題・検証

低次数多項式の全実根、重根・接触を含む交点の分離。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
