---
title: Bostan–Mori・有理母関数
documentation_of: //linear_recurrence/bostan_mori.hpp
---

# Bostan–Mori・有理母関数

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/linear_recurrence/index.html#bostan) / [実装](../../linear_recurrence/bostan_mori.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "linear_recurrence/bostan_mori.hpp"
int main() {
    std::cout << cp::bostan_mori({0, 1}, {1, -1, -1}, 10) << '\n';
    std::cout << cp::linear_recurrence_nth({0, 1}, {1, 1}, 10) << '\n';
}
```

[動く使用例](../../examples/linear_recurrence/bostan_mori.cpp)。ライブラリのルートをincludeパスに指定します。

## API・使用条件

`bostan_mori<Mod=998244353,Root=3>(P,Q,uint64_t k) -> int`

[x^k]P(x)/Q(x)を返します。PとQは昇冪順のvector<int>。Qの定数項が非0である必要があり、空／0はdomain_error。零分子、定数分母、分子次数が分母以上の場合も使えます。

`linear_recurrence_nth<Mod,Root>(initial,c,uint64_t n) -> int`

initialとcは同じ非空長d。不一致・空はinvalid_argument。係数順は他の手法と同じです。n<dでは初期項を返し、NTT容量制限を適用しません。n≥dではd+1≤fps_max_size<Mod,Root>()を要求します。

これは[3.17の同じ実装](../../fps/bostan_mori.hpp)を公開する入口です。関数の再定義や別実装は追加せず、両方のヘッダを同時にincludeできます。旧パスとAPIも保持します。

Modは2〜2×10^9の素数、Rootは原始根。P/Qを直接指定する場合、正規化・trim後の両入力長はL=fps_max_size<Mod,Root>()以下。LはNTT容量の半分（容量1なら1）、既定2^22、法17/根3では8、法97/根5では16。超過はlength_error。k=0でも分母と容量を検査します。

全てC++17・namespace cp、入力は不変。負の係数も[0,Mod)へ正規化し、INT_MIN/MAXを扱います。項番号・回数は0始まりのuint64_t全域で、負数を渡さないでください。[共通仕様](README.md#共通仕様)も参照してください。

## 原理・計算量

a[t]=Σc[j]a[t−1−j]の母関数A(x)=Σa[t]x^tに、Q(x)=1−Σc[j]x^(j+1)を掛けると、d次以上が消えます。P=(initial*Q) mod x^dとしてA=P/Qです。

Q(x)Q(−x)は偶数次だけになります。分子P(x)Q(−x)からkと同じ偶奇の係数、分母から偶数次の係数を取り出し、kを半分にします。最後はP[0]/Q[0]。C++では3.16のNTTを使い、図では定義式の積とP/Qへの変換を表示します。

N=max(|trim(P)|,|trim(Q)|)、M(N)=O(N log(N+1))として、入力正規化O(|P|+|Q|)、係数抽出O(M(N) log(k+1)+log Mod)時間。作業領域はO(|P|+|Q|)。漸化式版は分子を作るO(M(d))を加えます。

剰余の加減乗算はO(1)、中間の積はlong longに収め、加算ごとに剰余を取ります。

## 使用する問題・検証範囲

[Library Checker: kth_term_of_linearly_recurrent_sequence](https://judge.yosupo.jp/problem/kth_term_of_linearly_recurrent_sequence) / [ジャッジ形式](../../verify/linear_recurrence/bostan_mori.test.cpp)。

[検証入口](../../verify/run_linear_recurrence.py)で6ヘッダの単独・逆順・混在include、複数翻訳単位、4使用例、5ジャッジ形式・320ケースを確認します。BMは9,233有限列を独立した連立方程式の最小可解次数と比較。600漸化式・160任意行列、Python任意精度3,504クエリで各重み、初期項の順、合成数法・法1、非NTT素数、負数・整数極値、uint64最大値、空・次数1・遅い不一致・係数の末尾0を検証します。

192次Kitamasa、2,048観測／最短次数1,024のBM、48×48密行列、16,384次Bostan–Moriと2^63番目の項も確認します。通常／ASan・UBSan・LeakSanitizer、debug/release。図解は3,446入力・55,104スナップショットとブラウザ操作を検証します。オンラインACは未確認です。
