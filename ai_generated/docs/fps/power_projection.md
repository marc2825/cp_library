---
documentation_of: //fps/power_projection.hpp
---

# Power Projection（Q-27）

[カテゴリ](README.md) / [Q全31項目](../polynomial_extensions/README.md) / [実装](../../fps/power_projection.hpp) / [操作できる図解](../../visualizations/polynomial_extensions/index.html#projection)

## API・使用条件

`power_projection<Mod,Root>(weights,g,count)` → count項、結果k=Σ weights[i][x^i]g^k。

weights長が打切り精度。gの定数は非零でも可。空weightsは全0、count=0は空。内部二変数積がNTT容量を超えるとlength_error。

C++17、namespace `cp`。係数列は昇冪です。多項式を正規化するAPIでは末尾0を除き、ゼロ多項式を空配列で返します。FPS・畳み込み・変換の出力長は各APIの精度や入力長に従います。通常のmod係数は負数を正規化して返します。個別ヘッダと所属カテゴリの `all.hpp` から利用できます。NTTを共有するAPIの既定は `Mod=998244353, Root=3`、元のNTTの素数・原始根・容量条件を引き継ぎます。64bit/GF/複素数の係数型と規約は上記の専用APIに従います。

入力形状の不一致は `invalid_argument`、代数的な前提違反は `domain_error`、長さ・明示的予算超過は `length_error` です。NTT容量検査が先に行われる場合は代数的前提の検査より先に `length_error` となります。メモリ割当失敗は標準例外に従います。

## 原理・計算量

[x^(n−1)] reverse(weights)/(1−yg(x))を二変数Bostan–Moriで縮約。x次数を半分、y次数を倍にする。

O(M(n) log n+M(count))時間、保持する段のplanを含めO(n log n+count)領域。既存BSGSのO(n²)項を除去。 ここで `M(n)` はNTT多項式積の時間です。実用上の上限は計算量・パディング・領域から判断してください。

## 使用例

[コンパイルして動かすソース](../../examples/fps/power_projection.cpp)。

```cpp
#include "fps/power_projection.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() { print(cp::power_projection({1, 2, 3, 4}, {0, 1, 1}, 4)); }
```

出力:

```text
1 5 11 4
```

## 使用する問題・検証範囲

通常FPSの双対合成、冪の係数に対する重み付き集約に使用します。

[Qの比較検証](../polynomial_extensions/README.md#検証)で、通常のDebug/Release、libstdc++ debug mode、ASan/UBSanの対象にしています。使用例の出力・単独/重複include・全カテゴリ混在・複数翻訳単位・図解モデルの参照値を同じrunnerで確認します。数学的な比較対象と境界条件は検証索引に記載しています。オンライン提出ACは未確認です。

[一次資料と設計判断](../polynomial_extensions/references.md)。図解は小規模入力用の説明モデルで、高速実装と異なる手順を使う箇所は各ステップと[対応範囲](../../visualizations/polynomial_extensions/README.md)に記載しています。
