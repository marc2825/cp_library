---
title: Alias法・重み付き抽出・連続分布
documentation_of: //heuristic_search/distribution_sampling.hpp
---

# E-21 Alias法・重み付き抽出・連続分布

[一覧](README.md) / [ヘッダ](../../heuristic_search/distribution_sampling.hpp) / [実行例](../../examples/search_parsing_extensions/distribution_sampling.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#distribution_sampling) / [出典](references.md)

Alias列を大小の重みで埋める。非復元抽出は指数時計、連続分布は逆関数とBox–Muller変換を使う。

## APIと条件

AliasSampler、weighted_sample_without_replacement、NormalSampler、sample_exponential、sample_truncated_exponential。

Aliasはuint64整数重み・総和がuint64内で正、構築に128bit。非復元は正重みから逐次比例抽出と同じ順序分布。連続分布は52bit中点一様乱数とdoubleの近似。正規乱数の予備値・乱数消費を公開契約にする。

AliasSamplerは静的なuint64_t重みを受けます。空/総和0はinvalid_argument、総和overflowはoverflow_error。ゼロ重みを許します。総和Sとn*w_iを整数で扱い、thresholds()/aliases()は列の主IDへ進む境界と代替IDです。choose(column,coin)はcolumn<n,coin<Sを要求し、sample(Random&)は独立に列とcoinを選びます。整数のuniformを2回呼び、棄却によるraw消費数は可変です。

weighted_sample_without_replacement(weights,k,rng)はdoubleの有限非負重み、k≤正重み数を要求します。正重みごとに1rawを使い、log(−log(U))−log(w)の小さい順に返します。同値はID順。k=0は消費0。これは各回に残った重みに比例して選ぶ順序分布で、各要素の最終包含確率が元重みに比例するという意味ではありません。

NormalSampler::sample(rng,mean=0,standard_deviation=1)はBox–Mullerで2個作り、次回分を保存します。通常の2呼び出しでrawを2回（初回2・次回0）消費。標準偏差0はmeanを返し、rawも予備値も消費しません。reset()は予備値を捨てます。異なるmean/stddevでも予備値は標準正規として再利用できます。

sample_exponential(rate,rng)はrate>0、sample_truncated_exponential(rate,lower,upper,rng)は0≤lower≤upperで有限の区間を受けます。後者は条件付き分布で、単純なclampとは異なります。各1raw、区間幅0は消費0。極小rate×幅では逆CDFのTaylor展開を使います。連続値の生成には上位52bitの中点一様乱数を使用し、両端0/1を避けます。無限の裾は有限精度で切れ、整数Aliasのような厳密性は主張しません。結果が非有限ならoverflow_errorです。

## 計算量・用途

Alias前処理O(n)、抽出は整数の棄却法を含み期待O(1)。非復元k個はO(n log(k+2))・O(k)領域。正規/指数/切断指数はO(1)時間。

問題例・用途: 静的な非一様候補抽出、候補の非復元抽出、近傍生成の連続乱数。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "heuristic_search/distribution_sampling.hpp"
#include <iostream>
using namespace cp;
int main() {
    AliasSampler sampler({1, 3, 2});
    for (std::size_t i = 0; i < sampler.size(); ++i)
        std::cout << (i ? " " : "") << sampler.thresholds()[i];
    std::cout << "\n";
    for (std::size_t i = 0; i < sampler.size(); ++i)
        std::cout << (i ? " " : "") << sampler.aliases()[i];
    std::cout << "\n";
}
```

出力:

```text
3 6 3
2 1 1
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
