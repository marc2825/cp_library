---
documentation_of: //fps/polynomial_factorization.hpp
---

# 有限体上の多項式因数分解（Q-03）

[カテゴリ](README.md) / [Q全31項目](../polynomial_extensions/README.md) / [実装](../../fps/polynomial_factorization.hpp) / [操作できる図解](../../visualizations/polynomial_extensions/index.html#factorization)

## API・使用条件

polynomial_square_free_factorization、polynomial_distinct_degree_factorization、`polynomial_factorization<Mod>(f,seed=0)`。最終結果はleading_coefficientと{polynomial,multiplicity}列。

素数Mod。ゼロ多項式はdomain_error。定数は因子なし。DDF入口はmonicかつsquare-freeが必要。最終因子は次数・係数の辞書順。seedで再現可能なLas Vegas法。

C++17、namespace `cp`。係数列は昇冪です。多項式を正規化するAPIでは末尾0を除き、ゼロ多項式を空配列で返します。FPS・畳み込み・変換の出力長は各APIの精度や入力長に従います。通常のmod係数は負数を正規化して返します。個別ヘッダと所属カテゴリの `all.hpp` から利用できます。NTTを共有するAPIの既定は `Mod=998244353, Root=3`、元のNTTの素数・原始根・容量条件を引き継ぎます。64bit/GF/複素数の係数型と規約は上記の専用APIに従います。

入力形状の不一致は `invalid_argument`、代数的な前提違反は `domain_error`、長さ・明示的予算超過は `length_error` です。NTT容量検査が先に行われる場合は代数的前提の検査より先に `length_error` となります。メモリ割当失敗は標準例外に従います。

## 原理・計算量

微分とのGCDで重複を分け、微分0ならp乗根へ再帰。Frobeniusで次数別分解し、奇標数は二次指標、標数2はtraceで分割する。

素朴除算で剰余積はO(D²)。DDFはO(D³ log Mod)の保守的上限、EDFは乱択試行に依存し決定的時間上限なし。領域O(D²)以下。 ここで `M(n)` はNTT多項式積の時間です。実用上の上限は計算量・パディング・領域から判断してください。

## 使用例

[コンパイルして動かすソース](../../examples/fps/polynomial_factorization.cpp)。

```cpp
#include "fps/polynomial_factorization.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    auto f = cp::polynomial_factorization<3>({2, 1, 0, 1, 2});
    std::cout << f.leading_coefficient << "\n";
    for (const auto &part : f.factors) {
        print(part.polynomial);
        std::cout << part.multiplicity << "\n";
    }
}
```

出力:

```text
2
2 1
4
```

## 使用する問題・検証範囲

有限体多項式の既約因子、誤り訂正符号の代数計算。

[Qの比較検証](../polynomial_extensions/README.md#検証)で、通常のDebug/Release、libstdc++ debug mode、ASan/UBSanの対象にしています。使用例の出力・単独/重複include・全カテゴリ混在・複数翻訳単位・図解モデルの参照値を同じrunnerで確認します。数学的な比較対象と境界条件は検証索引に記載しています。オンライン提出ACは未確認です。

[一次資料と設計判断](../polynomial_extensions/references.md)。図解は小規模入力用の説明モデルで、高速実装と異なる手順を使う箇所は各ステップと[対応範囲](../../visualizations/polynomial_extensions/README.md)に記載しています。
