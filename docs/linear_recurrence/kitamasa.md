---
title: Kitamasa（きたまさ法）
documentation_of: //linear_recurrence/kitamasa.hpp
---

# Kitamasa（きたまさ法）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/linear_recurrence/index.html#kitamasa) / [実装](../../linear_recurrence/kitamasa.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "linear_recurrence/kitamasa.hpp"
int main() {
    std::cout << cp::kitamasa({0, 1}, {1, 1}, 10) << '\n';
    for (int x : cp::kitamasa_coefficients<17>({1, 1}, 10)) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/linear_recurrence/kitamasa.cpp)。ライブラリのルートをincludeパスに指定します。

## API・使用条件

`kitamasa<Mod=998244353>(initial,c,uint64_t n) -> int`

initialとcは同じ非空長dで、c[0]は直前の項に掛ける係数です。空や長さ違いはinvalid_argument。n<dは初期項を正規化して返します。係数の末尾0もdに含めます。

`kitamasa_coefficients<Mod>(c,uint64_t n) -> vector<int>`

X^nをF(X)=X^d−c[0]X^(d−1)−…−c[d−1]で割った剰余を**正確にd個**返します。戻り値wはa[n]=Σw[i]*initial[i]の重みで、初期項の昇順です。cは非空。n=0は{1,0,…}、d=1ではX≡c[0]、法1なら全て0。作業配列2d−1がvectorの上限を超える場合はlength_errorです。

法は1〜2×10^9で、合成数法・法1を含みます。除算・逆元やNTTを使いません。複数の初期値に同じ漸化式と項番号を使う場合、重みを一度求めて内積を取れます。

全てC++17・namespace cp、入力は不変。負の係数も[0,Mod)へ正規化し、INT_MIN/MAXを扱います。項番号・回数は0始まりのuint64_t全域で、負数を渡さないでください。[共通仕様](README.md#共通仕様)も参照してください。

## 原理・計算量

漸化式によりX^d≡c[0]X^(d−1)+…+c[d−1]。剰余を掛け合わせると次数が最大2d−2になるため、高い次数からこの式で還元します。その積を二分累乗してX^n mod Fを得て、初期項に対する重みにします。

c[0]がX^(d−1)へ寄与することと、戻り値w[0]がa[0]に掛かることを区別します。一般の体を仮定せず、Fがモニックなので合成数法でも同じ還元が使えます。

O(d² log(n+1)+d)時間、O(d)補助領域。公開の重みAPIはn<dでもこの実装で計算します。kitamasa本体はn<dでO(1)。畳み込みを使った高速版はBostan–Moriを選んでください。

剰余の加減乗算はO(1)、中間の積はlong longに収め、加算ごとに剰余を取ります。

## 使用する問題・検証範囲

[Library Checker: kth_term_of_linearly_recurrent_sequence](https://judge.yosupo.jp/problem/kth_term_of_linearly_recurrent_sequence) / [ジャッジ形式](../../verify/linear_recurrence/kitamasa.test.cpp)。

[検証入口](../../verify/run_linear_recurrence.py)で6ヘッダの単独・逆順・混在include、複数翻訳単位、4使用例、5ジャッジ形式・320ケースを確認します。BMは9,233有限列を独立した連立方程式の最小可解次数と比較。600漸化式・160任意行列、Python任意精度3,504クエリで各重み、初期項の順、合成数法・法1、非NTT素数、負数・整数極値、uint64最大値、空・次数1・遅い不一致・係数の末尾0を検証します。

192次Kitamasa、2,048観測／最短次数1,024のBM、48×48密行列、16,384次Bostan–Moriと2^63番目の項も確認します。通常／ASan・UBSan・LeakSanitizer、debug/release。図解は3,446入力・55,104スナップショットとブラウザ操作を検証します。オンラインACは未確認です。
