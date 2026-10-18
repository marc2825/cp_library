---
title: オンライン・オフライン変換（CDQ）
documentation_of: //dp_optimization/online_offline_dp.hpp
---

# オンライン・オフライン変換（CDQ）

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#online_offline_dp) / [実装](../../dp_optimization/online_offline_dp.hpp) / [一次資料と設計](../b_12_15/references.md)

左半分を確定した後、その寄与を右半分へ一括転送してから右を確定するCDQテンプレートです。畳み込みDPの具体版も用意します。

## usage

```cpp
#include <iostream>
#include "dp_optimization/online_offline_dp.hpp"
int main() {
    auto dp=cp::online_offline_convolution_dp(std::vector<int>{1,0,0,0,0},std::vector<int>{0,1,1,1,1});
    std::cout << dp.back() << '\n';
}
```

出力:

```text
8
```

## API・使用条件

`cdq_online_offline(n,finish(i),transfer(l,m,r))`、`online_offline_convolution_dp<Mod,Root>(initial,kernel)`。具体版dp[i]=initial[i]+Σ_{j<i}dp[j]kernel[i-j] mod Mod、kernel[0]は無視。0始まり・半開区間。

## 仕組み・境界条件

transfer時に[l,m)は確定済み、[m,r)は未確定。各依存j<iを一度だけ担当する区間対へ分けます。任意のオンラインアルゴリズムを自動変換するものではなく、寄与をまとめて転送できるDPが対象です。畳み込み版は既存NTTの素数法・原始根・変換長上限に従います。

## 計算量

テンプレートはN回のfinishとmax(0,N−1)回のtransfer。各段の転送合計T(N)ならO(T(N) log N)。畳み込み版O(N log²N)時間、O(N)領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

dp[i]=初期値+過去dpとカーネルの畳み込み、依存を持つオフラインDP。 [動く小問題](../../examples/dp_optimization/online_offline_dp_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
