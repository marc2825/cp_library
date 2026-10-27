---
documentation_of: //convolution/ntt64.hpp
---

# Goldilocks 64bit NTT（Q-22）

[カテゴリ](README.md) / [Q全31項目](../polynomial_extensions/README.md) / [実装](../../convolution/ntt64.hpp) / [操作できる図解](../../visualizations/polynomial_extensions/index.html#ntt64)

## API・使用条件

`ntt64(vector<uint64_t>&,inverse=false)`、`convolution_ntt64(a,b)`、ntt64_modulus。

固定素数p=2^64−2^32+1、原始根7、長さ2冪≤2^32（実メモリ上限も適用）。GCC/Clangの128bit拡張。入力はpで正規化。

C++17、namespace `cp`。係数列は昇冪です。多項式を正規化するAPIでは末尾0を除き、ゼロ多項式を空配列で返します。FPS・畳み込み・変換の出力長は各APIの精度や入力長に従います。通常のmod係数は負数を正規化して返します。個別ヘッダと所属カテゴリの `all.hpp` から利用できます。NTTを共有するAPIの既定は `Mod=998244353, Root=3`、元のNTTの素数・原始根・容量条件を引き継ぎます。64bit/GF/複素数の係数型と規約は上記の専用APIに従います。

入力形状の不一致は `invalid_argument`、代数的な前提違反は `domain_error`、長さ・明示的予算超過は `length_error` です。NTT容量検査が先に行われる場合は代数的前提の検査より先に `length_error` となります。メモリ割当失敗は標準例外に従います。

## 原理・計算量

Goldilocks素数専用reducerと段ごとの単位根cache。任意の64bit法には対応しない。

O(N log N)時間・O(N)領域。乗算の剰余は整数除算を使わず、2^64≡2^32−1で還元。 ここで `M(n)` はNTT多項式積の時間です。実用上の上限は計算量・パディング・領域から判断してください。

## 使用例

[コンパイルして動かすソース](../../examples/convolution/ntt64.cpp)。

```cpp
#include "convolution/ntt64.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() { print(cp::convolution_ntt64({cp::ntt64_modulus - 1, 2}, {2, 3})); }
```

出力:

```text
18446744069414584319 1 6
```

## 使用する問題・検証範囲

64bit素数上の多項式積。高性能は専用還元の設計を指し、全環境での速度優位を保証しない。

[Qの比較検証](../polynomial_extensions/README.md#検証)で、通常のDebug/Release、libstdc++ debug mode、ASan/UBSanの対象にしています。使用例の出力・単独/重複include・全カテゴリ混在・複数翻訳単位・図解モデルの参照値を同じrunnerで確認します。数学的な比較対象と境界条件は検証索引に記載しています。オンライン提出ACは未確認です。

[一次資料と設計判断](../polynomial_extensions/references.md)。図解は小規模入力用の説明モデルで、高速実装と異なる手順を使う箇所は各ステップと[対応範囲](../../visualizations/polynomial_extensions/README.md)に記載しています。
