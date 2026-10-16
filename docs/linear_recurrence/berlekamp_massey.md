---
title: Berlekamp–Massey（最短漸化式の推定）
documentation_of: //linear_recurrence/berlekamp_massey.hpp
---

# Berlekamp–Massey（最短漸化式の推定）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/linear_recurrence/index.html#berlekamp) / [実装](../../linear_recurrence/berlekamp_massey.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "linear_recurrence/berlekamp_massey.hpp"
int main() {
    const std::vector<int> s{0, 1, 1, 2, 3, 5, 8, 13};
    for (int x : cp::berlekamp_massey(s)) std::cout << x << ' ';
    std::cout << '\n';
    std::cout << cp::linear_recurrence_from_prefix(s, 10) << '\n';
}
```

[動く使用例](../../examples/linear_recurrence/berlekamp_massey.cpp)。ライブラリのルートをincludeパスに指定します。

## API・使用条件

`berlekamp_massey<Mod=998244353>(samples) -> vector<int>`

Modは2以上2×10^9以下の**素数**です。合成数法・法1はコンパイル時に拒否します。NTTに適した法である必要はなく、1000000007も使えます。

有限の観測列sに対し、s[t]=Σc[j]*s[t−1−j] (L≤t<s.size())を満たす最短の長さLのcを返します。c[0]は直前の項です。空・全0列は空。**末尾0は次数を保つため残します**。例えば{1,0,0,0}の結果は{0}で、空へ縮めると最初の1を説明できません。複数解の場合の辞書順最小等は保証しません。

短い観測ではLが観測数の半分を超える場合があります。{0,0,1}の最短次数は3で、得られる規則は有限の観測を説明するものです。真の無限列が次数d以下の同次線形漸化式に従うと分かっていれば、先頭2d項でその無限列を再現する規則を求められます。観測だけから未知の将来を無条件に特定することはできません。

`linear_recurrence_from_prefix<Mod>(samples,uint64_t n) -> int`

nが観測範囲ならその値を返し、それ以外はBMが選ぶ漸化式をKitamasaで延長します。空・全0の観測は零列として延長。真の次数の上限と観測数の条件は上と同じです。

全てC++17・namespace cp、入力は不変。負の係数も[0,Mod)へ正規化し、INT_MIN/MAXを扱います。項番号・回数は0始まりのuint64_t全域で、負数を渡さないでください。[共通仕様](README.md#共通仕様)も参照してください。

## 原理・計算量

接続多項式C[0]=1を使い、delta=s[i]+ΣC[j]s[i−j]が0か確認します。0なら現在の規則で一致。不一致なら保存した式Bを失敗点からの距離だけずらし、過去の不一致量との比を掛けてCから引きます。必要なときはLをi+1−Lへ増やし、以前のCと不一致量を保存します。

Cは「左辺へ移項した係数」、公開するcは「右辺の係数」なので、返すときに符号を反転します。Lは末尾0を除いた多項式次数とは異なります。

観測数Sに対してO(S²+S log Mod)時間、O(S)補助領域。各不一致で逆元を剰余累乗により計算します。延長用APIは観測範囲内ならO(1)、それ以外はさらにO(L² log(n+1))時間・O(L)領域です。

剰余の加減乗算はO(1)、中間の積はlong longに収め、加算ごとに剰余を取ります。

## 使用する問題・検証範囲

[Library Checker: find_linear_recurrence](https://judge.yosupo.jp/problem/find_linear_recurrence) / [ジャッジ形式](../../verify/linear_recurrence/berlekamp_massey.test.cpp)。

[検証入口](../../verify/run_linear_recurrence.py)で6ヘッダの単独・逆順・混在include、複数翻訳単位、4使用例、5ジャッジ形式・320ケースを確認します。BMは9,233有限列を独立した連立方程式の最小可解次数と比較。600漸化式・160任意行列、Python任意精度3,504クエリで各重み、初期項の順、合成数法・法1、非NTT素数、負数・整数極値、uint64最大値、空・次数1・遅い不一致・係数の末尾0を検証します。

192次Kitamasa、2,048観測／最短次数1,024のBM、48×48密行列、16,384次Bostan–Moriと2^63番目の項も確認します。通常／ASan・UBSan・LeakSanitizer、debug/release。図解は3,446入力・55,104スナップショットとブラウザ操作を検証します。オンラインACは未確認です。
