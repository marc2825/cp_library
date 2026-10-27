---
documentation_of: //fps/polynomial_matrix.hpp
---

# 多項式行列式・評価行列Prefix Product（Q-24）

[カテゴリ](README.md) / [Q全31項目](../polynomial_extensions/README.md) / [実装](../../fps/polynomial_matrix.hpp) / [操作できる図解](../../visualizations/polynomial_extensions/index.html#polynomial_matrix)

## API・使用条件

`PolynomialMatrix=vector<vector<vector<int>>>`、`polynomial_matrix_determinant<Mod>(a)`、`polynomial_matrix_prefix_product<Mod,Root>(a,start,count,max_block_degree=65536)`。

正方行列。空行列式1。prefixはM(start+count−1)…M(start)の順序、count0は単位行列。引数は体へ還元して評価。block次数と評価点数をbudget以内に制限。

C++17、namespace `cp`。係数列は昇冪です。多項式を正規化するAPIでは末尾0を除き、ゼロ多項式を空配列で返します。FPS・畳み込み・変換の出力長は各APIの精度や入力長に従います。通常のmod係数は負数を正規化して返します。個別ヘッダと所属カテゴリの `all.hpp` から利用できます。NTTを共有するAPIの既定は `Mod=998244353, Root=3`、元のNTTの素数・原始根・容量条件を引き継ぎます。64bit/GF/複素数の係数型と規約は上記の専用APIに従います。

入力形状の不一致は `invalid_argument`、代数的な前提違反は `domain_error`、長さ・明示的予算超過は `length_error` です。NTT容量検査が先に行われる場合は代数的前提の検査より先に `length_error` となります。メモリ割当失敗は標準例外に従います。

## 原理・計算量

係数の多項式そのものを保持した分数なし消去と、連続評価点のbaby-step/giant-step積。単なる一地点評価のAPIではない。

行列式はBareiss、行列サイズr・途中次数Dで保守的O(r³D²)。prefixはb≈√(count/degree)、多項式block構築とr²回の多点評価、O(r³count/b)の行列積。定数行列は二分累乗。 ここで `M(n)` はNTT多項式積の時間です。実用上の上限は計算量・パディング・領域から判断してください。

## 使用例

[コンパイルして動かすソース](../../examples/fps/polynomial_matrix.cpp)。

```cpp
#include "fps/polynomial_matrix.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    cp::PolynomialMatrix a{{{0, 1}, {1}}, {{1}, {0, 1}}};
    print(cp::polynomial_matrix_determinant(a));
    auto p = cp::polynomial_matrix_prefix_product(a, 0, 3);
    for (const auto &row : p)
        print(row);
}
```

出力:

```text
998244352 0 1
3 3
3 3
```

## 使用する問題・検証範囲

多項式行列の特異性、P-recursiveの大きな添字。

[Qの比較検証](../polynomial_extensions/README.md#検証)で、通常のDebug/Release、libstdc++ debug mode、ASan/UBSanの対象にしています。使用例の出力・単独/重複include・全カテゴリ混在・複数翻訳単位・図解モデルの参照値を同じrunnerで確認します。数学的な比較対象と境界条件は検証索引に記載しています。オンライン提出ACは未確認です。

[一次資料と設計判断](../polynomial_extensions/references.md)。図解は小規模入力用の説明モデルで、高速実装と異なる手順を使う箇所は各ステップと[対応範囲](../../visualizations/polynomial_extensions/README.md)に記載しています。
