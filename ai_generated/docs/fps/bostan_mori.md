---
title: Bostan–Mori・線形漸化式の第n項
documentation_of: //fps/bostan_mori.hpp
---

# Bostan–Mori・線形漸化式の第n項

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/fps/index.html#bostan) / [実装](../../fps/bostan_mori.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "fps/bostan_mori.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print({cp::bostan_mori<17, 3>({0, 1}, {1, -1, -1}, 10)});
    print({cp::linear_recurrence_nth<17, 3>({0, 1}, {1, 1}, 10)});
}
```

[動く使用例](../../examples/fps/bostan_mori.cpp)。ルートをincludeパスに追加します。

## API・使用条件

`bostan_mori<Mod,Root>(P,Q,uint64_t k) -> int`

0始まりの[x^k]P(x)/Q(x)を返します。Qの定数項は非0で、空や0ならdomain_error。deg(P)≥deg(Q)、定数分母、零分子、末尾0も扱います。正規化・trim後の両入力の長さ≤Lを要求します。k=0でも分母と容量を検査します。kはuint64_t全域、負数は渡さないでください。

`linear_recurrence_nth<Mod,Root>(initial,c,uint64_t n) -> int`

a[t]=Σ_(j=0..d−1)c[j]*a[t−1−j] (t≥d) のa[n]を返します。initial={a[0],…,a[d−1]}とcは同じ非空長。不一致や空はinvalid_argument。n<dなら該当する初期値だけを正規化して返し、容量制限は適用しません。それ以外はd+1≤Lが必要です。

全て昇冪順の `vector<int>` を入力とし、入力を変更しません。負の係数も[0,Mod)へ正規化します。Modの既定値998244353、Rootの既定値3。[共通仕様・精度上限L](README.md#共通仕様)を参照してください。

## 原理・計算量

Q(x)Q(−x)は偶数次だけになります。P(x)Q(−x)からkと同じ偶奇の項、分母から偶数次の項を抜き、kを半分にします。最後はP₀/Q₀。漸化式はQ=1−c₀x−…−c_(d−1)x^d、P=(initial*Q) mod x^dに変換します。

N=max(|trim(P)|,|trim(Q)|)。入力正規化O(|P|+|Q|)、Bostan–MoriはO(M(N) log(k+1)+log Mod)時間、O(N)作業領域（元入力のコピーも含めるとO(|P|+|Q|)）。漸化式ラッパは母関数の分子を作るO(M(d))を加えます。

ここでM(N)=O(N log(N+1))はNTTによる積の費用です。法の乗算をO(1)として数えます。

## 使用する問題・検証範囲

[Library Checker: kth_term_of_linearly_recurrent_sequence](https://judge.yosupo.jp/problem/kth_term_of_linearly_recurrent_sequence)。[ジャッジ形式のプログラム](../../verify/fps/bostan_mori.test.cpp)を用意しています。

[検証入口](../../verify/run_fps.py)は14ヘッダの単独／逆順／混在includeと複数翻訳単位、11使用例、10ジャッジ形式・480ケースを確認します。1,093三値級数を9種類の出力精度で、240ランダム多項式、300漸化式、Python任意精度4,352クエリを独立の定義式・係数漸化式・Horner・特性多項式の剰余と比較します。空、精度0、先頭と末尾0、平方根なし、重複点、除数0、int/long long/uint64の極値、法2/3/17/97/998244353/1224736769を含みます。

65,536項のNewton法・Taylor Shift・高速除算、8,192点の評価／補間、4,096次の漸化式とuint64の巨大項も確認します。通常とASan/UBSan、debug/releaseで実行します。図解モデルとブラウザ操作にも独立検証を用意しています。オンラインACは未確認です。

[3.18の説明・他手法との比較](../linear_recurrence/bostan_mori.md) / [漸化式からP/Qを作る図解](../../visualizations/linear_recurrence/index.html#bostan)。旧APIとincludeパスは保持しています。
