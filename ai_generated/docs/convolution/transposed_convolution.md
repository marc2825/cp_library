---
documentation_of: //convolution/transposed_convolution.hpp
---

# 転置原理・固定畳み込みの共有前計算（Q-06）

[カテゴリ](README.md) / [Q全31項目](../polynomial_extensions/README.md) / [実装](../../convolution/transposed_convolution.hpp) / [操作できる図解](../../visualizations/polynomial_extensions/index.html#transpose)

## API・使用条件

`TransposedConvolutionPlan<Mod,Root>(kernel,input_size,output_size)`、`apply(x)`、`transpose(weights)`、`input_size()`、`output_size()`。

kernelと入力長は非零、出力長は0〜全積長。applyとtransposeは正確なサイズを要求する。通常の係数内積で転置し、複素共役ではない。

C++17、namespace `cp`。係数列は昇冪です。多項式を正規化するAPIでは末尾0を除き、ゼロ多項式を空配列で返します。FPS・畳み込み・変換の出力長は各APIの精度や入力長に従います。通常のmod係数は負数を正規化して返します。個別ヘッダと所属カテゴリの `all.hpp` から利用できます。NTTを共有するAPIの既定は `Mod=998244353, Root=3`、元のNTTの素数・原始根・容量条件を引き継ぎます。64bit/GF/複素数の係数型と規約は上記の専用APIに従います。

入力形状の不一致は `invalid_argument`、代数的な前提違反は `domain_error`、長さ・明示的予算超過は `length_error` です。NTT容量検査が先に行われる場合は代数的前提の検査より先に `length_error` となります。メモリ割当失敗は標準例外に従います。

## 原理・計算量

切り詰めToeplitz作用と、その転置の相関計算を、周波数添字の反転で実装。内積〈Ax,w〉=〈x,Aᵀw〉を検証する。

構築・各作用O(N log N)、保存O(N)。kernelのNTT値を両方向で共有。 ここで `M(n)` はNTT多項式積の時間です。実用上の上限は計算量・パディング・領域から判断してください。

## 使用例

[コンパイルして動かすソース](../../examples/convolution/transposed_convolution.cpp)。

```cpp
#include "convolution/transposed_convolution.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    cp::TransposedConvolutionPlan<> plan({1, 2}, 3, 3);
    print(plan.apply({3, 4, 5}));
    print(plan.transpose({7, 8, 9}));
}
```

出力:

```text
3 10 13
23 26 9
```

## 使用する問題・検証範囲

重み付き係数集約、逆伝播、線形変換の双対。

[Qの比較検証](../polynomial_extensions/README.md#検証)で、通常のDebug/Release、libstdc++ debug mode、ASan/UBSanの対象にしています。使用例の出力・単独/重複include・全カテゴリ混在・複数翻訳単位・図解モデルの参照値を同じrunnerで確認します。数学的な比較対象と境界条件は検証索引に記載しています。オンライン提出ACは未確認です。

[一次資料と設計判断](../polynomial_extensions/references.md)。図解は小規模入力用の説明モデルで、高速実装と異なる手順を使う箇所は各ステップと[対応範囲](../../visualizations/polynomial_extensions/README.md)に記載しています。
