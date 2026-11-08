---
documentation_of: //fps/differential_equation.hpp
---

# FPSの微分方程式（Q-33）

[カテゴリ](README.md) / [Q全31項目](../polynomial_extensions/README.md) / [実装](../../fps/differential_equation.hpp) / [操作できる図解](../../visualizations/polynomial_extensions/index.html#ode)

## API・使用条件

`fps_solve_linear_ode(a,b,initial,n)`、`fps_solve_polynomial_ode(coefficients,initial,n)`。後者は `y'=Σcoefficients[j](x)y^j`。

係数は素数体、n≤ModかつFPS容量内。初期値を指定する一階方程式。特異なy'係数や連立方程式はこのAPIの範囲外。

C++17、namespace `cp`。係数列は昇冪です。多項式を正規化するAPIでは末尾0を除き、ゼロ多項式を空配列で返します。FPS・畳み込み・変換の出力長は各APIの精度や入力長に従います。通常のmod係数は負数を正規化して返します。個別ヘッダと所属カテゴリの `all.hpp` から利用できます。NTTを共有するAPIの既定は `Mod=998244353, Root=3`、元のNTTの素数・原始根・容量条件を引き継ぎます。64bit/GF/複素数の係数型と規約は上記の専用APIに従います。

入力形状の不一致は `invalid_argument`、代数的な前提違反は `domain_error`、長さ・明示的予算超過は `length_error` です。NTT容量検査が先に行われる場合は代数的前提の検査より先に `length_error` となります。メモリ割当失敗は標準例外に従います。

## 原理・計算量

積分因子による線形解とNewton線形化。補正δ'=F_yδ+F−y'、δ(0)=0を倍増精度で解く。

線形はO(M(n))、y次数dの多項式右辺は保守的にO(d M(n) log n)、領域O(dn+n)。 ここで `M(n)` はNTT多項式積の時間です。実用上の上限は計算量・パディング・領域から判断してください。

## 使用例

[コンパイルして動かすソース](../../examples/fps/differential_equation.cpp)。

```cpp
#include "fps/differential_equation.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() { print(cp::fps_solve_polynomial_ode({{}, {}, {1}}, 1, 6)); }
```

出力:

```text
1 1 1 1 1 1
```

## 使用する問題・検証範囲

漸化式から得た生成関数、Riccati型方程式。

[Qの比較検証](../polynomial_extensions/README.md#検証)で、通常のDebug/Release、libstdc++ debug mode、ASan/UBSanの対象にしています。使用例の出力・単独/重複include・全カテゴリ混在・複数翻訳単位・図解モデルの参照値を同じrunnerで確認します。数学的な比較対象と境界条件は検証索引に記載しています。オンライン提出ACは未確認です。

[一次資料と設計判断](../polynomial_extensions/references.md)。図解は小規模入力用の説明モデルで、高速実装と異なる手順を使う箇所は各ステップと[対応範囲](../../visualizations/polynomial_extensions/README.md)に記載しています。
