---
title: Q(ω)の超現実数・無限小
documentation_of: //game_theory/omega_rational.hpp
---

# B-20 Q(ω)の超現実数・無限小

[カテゴリ](README.md) / [実装](../../game_theory/omega_rational.hpp) / [操作できる図解](../../visualizations/optimization_extensions/index.html#omega_rational) / [一次資料](references.md)

ωを全ての有理定数より大きい正数とし、既約有理関数の最高次係数で符号を決める。

## API・条件

ユーザー承認済みのQ(ω)を `OmegaRational` で表します。ωは全ての有理定数より大きい正の超現実数です。係数が任意精度有理数である多項式p,qを使い、p(ω)/q(ω)を有限データで保持します。qは非零。係数は昇べき順で、末尾0の削除・多項式GCDによる約分・分母最高次係数1への正規化を行います。

有理定数の構築、omega()、infinitesimal()=1/ω、加減乗除、六つの比較、sign()、bool変換を公開します。numerator()/denominator()で正規化した係数列、to_string()でwを使った表示を得られます。0除算はdomain_errorです。コピーは値を保持し、move元は0へ戻します。自分自身への代入や演算も可能です。

分母をmonicへ揃えると、符号は分子の最高次係数の符号です。従って例えばω>10^100、0<1/ω<1/10^100、(ω²−1)/(ω−1)=ω+1を厳密に扱えます。

is_finite()は分子次数≤分母次数、is_infinitesimal()は分子次数<分母次数（0も含む）です。standard_part()は有限値なら有理数の標準部分、無限大の大きさならnullopt。outcome()は数として正ならleft_wins、負ならright_wins、0ならprevious_winsを返し、既存PartizanOutcomeと共有します。この部分体の数はfuzzyではありません。

一般の{L|R}の無限選択肢を生成・列挙するAPIではありません。√ω、ω^ω、任意の無限級数、非数のゲーム値は表せません。通常の超現実数の和・積なので可換です。順序数の加法やnim-sum/nim-productとは異なります。有限short gameの既存[PartizanGameArena](../game_theory/partizan_game.md)は保持しています。

有理多項式のtrim・除算・GCDはYのSturm実根分離から共通部品へ抽出しました。任意精度係数・次数の成長は呼び出し側で管理してください。

## 計算量

次数dで基本の積・除算・GCDはO(d²)有理演算規模。係数のbit成長と結果次数に依存する。

## 使用例

```cpp
#include "game_theory/omega_rational.hpp"
#include <iostream>
using namespace cp;
int main() {
    const auto w = OmegaRational::omega(), epsilon = OmegaRational::infinitesimal();
    std::cout << ((w * w - 1) / (w - 1) == w + 1) << " " << (epsilon > 0) << " "
              << *(1 + epsilon).standard_part() << "\n";
}
```

出力:

```text
1 1 1/1
```

## 使用する問題・検証

無限大・無限小を含む数の比較、数としてのゲーム値の和・差、記号的な漸近順序。 オンラインACは未確認です。

[単体・全探索比較](../../verify/optimization_extensions/unit_test.cpp)、[独立Python比較](../../verify/run_optimization_extensions.py)、[使用例と参照値](../../verify/optimization_extensions/manifest.json)で検証します。検証コマンドと図解との相違は[カテゴリREADME](README.md)を参照してください。
