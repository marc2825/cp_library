---
documentation_of: //convolution/rader_fft.hpp
---

# Rader FFT（Q-20）

[カテゴリ](README.md) / [Q全31項目](../polynomial_extensions/README.md) / [実装](../../convolution/rader_fft.hpp) / [操作できる図解](../../visualizations/polynomial_extensions/index.html#rader)

## API・使用条件

`rader_fft(vector<complex<Real>>&,inverse=false)`。

長さは素数2〜2×10^9。順変換はexp(+2πijk/n)、逆は負符号と1/n。有限入力のみ、近似値。

C++17、namespace `cp`。係数列は昇冪です。多項式を正規化するAPIでは末尾0を除き、ゼロ多項式を空配列で返します。FPS・畳み込み・変換の出力長は各APIの精度や入力長に従います。通常のmod係数は負数を正規化して返します。個別ヘッダと所属カテゴリの `all.hpp` から利用できます。NTTを共有するAPIの既定は `Mod=998244353, Root=3`、元のNTTの素数・原始根・容量条件を引き継ぎます。64bit/GF/複素数の係数型と規約は上記の専用APIに従います。

入力形状の不一致は `invalid_argument`、代数的な前提違反は `domain_error`、長さ・明示的予算超過は `length_error` です。NTT容量検査が先に行われる場合は代数的前提の検査より先に `length_error` となります。メモリ割当失敗は標準例外に従います。

## 原理・計算量

非零添字を原始根の冪に並べ替え、巡回畳み込みへ帰着。

O(n log n)時間・O(n)領域。素数・原始根の準備は試し割り。 ここで `M(n)` はNTT多項式積の時間です。実用上の上限は計算量・パディング・領域から判断してください。

## 使用例

[コンパイルして動かすソース](../../examples/convolution/rader_fft.cpp)。

```cpp
#include "convolution/rader_fft.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    std::vector<std::complex<double>> a{{1, 0}, {2, 0}, {3, 0}};
    cp::rader_fft(a);
    for (auto x : a)
        std::cout << std::llround(x.real() * 1000000) << " " << std::llround(x.imag() * 1000000) << "\n";
}
```

出力:

```text
6000000 0
-1500000 -866025
-1500000 866025
```

## 使用する問題・検証範囲

素数長のスペクトル解析・DFT。

[Qの比較検証](../polynomial_extensions/README.md#検証)で、通常のDebug/Release、libstdc++ debug mode、ASan/UBSanの対象にしています。使用例の出力・単独/重複include・全カテゴリ混在・複数翻訳単位・図解モデルの参照値を同じrunnerで確認します。数学的な比較対象と境界条件は検証索引に記載しています。オンライン提出ACは未確認です。

[一次資料と設計判断](../polynomial_extensions/references.md)。図解は小規模入力用の説明モデルで、高速実装と異なる手順を使う箇所は各ステップと[対応範囲](../../visualizations/polynomial_extensions/README.md)に記載しています。
