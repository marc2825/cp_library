---
title: SA受理判定の対数テーブル
documentation_of: //heuristic_search/fast_annealing_acceptance.hpp
---

# E-04 SA受理判定の対数テーブル

[一覧](README.md) / [ヘッダ](../../heuristic_search/fast_annealing_acceptance.hpp) / [実行例](../../examples/search_parsing_extensions/fast_annealing_acceptance.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#fast_annealing_acceptance) / [出典](references.md)

均等な確率区間の中点について−log(u)を前計算し、loss<T×閾値で採否を決める。

## APIと条件

FastAnnealingAcceptance(bits=12)、operator()(loss,temperature,rng)、accept_index、simulated_annealing_delta_acceptance。

bitsは1〜24、有限のloss≥0・temperature>0。悪化時はraw乱数1回、loss=0なら0回。中央値量子化による確率誤差≤2^(-bits-1)に浮動丸めを加える。通常SAの既存exp採否は保持。

N=2^bits、u_i=(i+1/2)/N、table[i]=−log(u_i)。一様なiに対してloss<T×table[i]と比較します。厳密な実数計算なら、区間中点を数えるCDFのずれは1/(2N)以下です。long doubleのlog/乗算にはさらに丸め誤差があります。極小確率は0へ量子化され得ます。

operator()(loss,T,Random&)はrawの上位bitsを使います。accept_index(loss,T,i)は乱数なしで特定binの採否を返します。thresholds()/bits()/probability_error_bound()で設定を確認できます。既定12bitは4096エントリ、確率誤差上限1/8192。テーブルはstd::ref(table)で渡すとコピーを避けられます。

simulated_annealing_delta_acceptance(initial,evaluate,propose,delta,apply,undo,rng,accept_worse,options)は悪化時だけaccept_worse(loss,T,rng)を呼びます。改善・同点は無条件受理です。従来simulated_annealing_deltaのexp判定と乱数順序は保持しました。高速化の実測条件と結果は[benchmark](benchmark.md)を参照してください。

## 計算量・用途

前計算O(2^bits)時間/領域、採否O(1)でlog/expなし。性能優位は環境と入力に依存。

問題例・用途: 悪化moveの採否がボトルネックになる固定回数SA。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "heuristic_search/fast_annealing_acceptance.hpp"
#include <iostream>
using namespace cp;
int main() {
    FastAnnealingAcceptance accept(4);
    int count = 0;
    for (std::size_t i = 0; i < accept.thresholds().size(); ++i)
        count += accept.accept_index(.7L, 1, i);
    std::cout << count << " " << accept.thresholds().size() << "\n";
}
```

出力:

```text
8 16
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
