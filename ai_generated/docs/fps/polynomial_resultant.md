---
documentation_of: //fps/polynomial_resultant.hpp
---

# 終結式・判別式（Q-02）

[カテゴリ](README.md) / [Q全31項目](../polynomial_extensions/README.md) / [実装](../../fps/polynomial_resultant.hpp) / [操作できる図解](../../visualizations/polynomial_extensions/index.html#resultant)

## API・使用条件

`polynomial_resultant<Mod>(a,b)`、`polynomial_discriminant<Mod>(f)` → int。

素数Mod。ゼロ入力の終結式は0、非零定数同士は1。判別式はゼロ多項式0、非零定数・一次1。標数2も同じ代数的定義。

C++17、namespace `cp`。係数列は昇冪です。多項式を正規化するAPIでは末尾0を除き、ゼロ多項式を空配列で返します。FPS・畳み込み・変換の出力長は各APIの精度や入力長に従います。通常のmod係数は負数を正規化して返します。個別ヘッダと所属カテゴリの `all.hpp` から利用できます。NTTを共有するAPIの既定は `Mod=998244353, Root=3`、元のNTTの素数・原始根・容量条件を引き継ぎます。64bit/GF/複素数の係数型と規約は上記の専用APIに従います。

入力形状の不一致は `invalid_argument`、代数的な前提違反は `domain_error`、長さ・明示的予算超過は `length_error` です。NTT容量検査が先に行われる場合は代数的前提の検査より先に `length_error` となります。メモリ割当失敗は標準例外に従います。

## 原理・計算量

剰余への置換時に先頭係数の冪と交換符号を掛ける。判別式は(-1)^(d(d-1)/2) Res(f,f')/lc(f)。

通常の多項式除算によるEuclid法、O(D²)時間・O(D)領域。 ここで `M(n)` はNTT多項式積の時間です。実用上の上限は計算量・パディング・領域から判断してください。

## 使用例

[コンパイルして動かすソース](../../examples/fps/polynomial_resultant.cpp)。

```cpp
#include "fps/polynomial_resultant.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    std::cout << cp::polynomial_resultant({2, -3, 1}, {-3, 1}) << "\n";
    std::cout << cp::polynomial_discriminant({2, -3, 1}) << "\n";
}
```

出力:

```text
2
1
```

## 使用する問題・検証範囲

共通根、重根の検出。

[Qの比較検証](../polynomial_extensions/README.md#検証)で、通常のDebug/Release、libstdc++ debug mode、ASan/UBSanの対象にしています。使用例の出力・単独/重複include・全カテゴリ混在・複数翻訳単位・図解モデルの参照値を同じrunnerで確認します。数学的な比較対象と境界条件は検証索引に記載しています。オンライン提出ACは未確認です。

[一次資料と設計判断](../polynomial_extensions/references.md)。図解は小規模入力用の説明モデルで、高速実装と異なる手順を使う箇所は各ステップと[対応範囲](../../visualizations/polynomial_extensions/README.md)に記載しています。
