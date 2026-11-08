---
title: Fast Set / Predecessor Set
documentation_of: //static_query/fast_set.hpp
---

# Fast Set / Predecessor Set

固定整数宇宙の集合を64bitの階層ビット列で管理。

## API・使用条件

FastSet(n)、insert(x)/erase(x)->変更有無、contains(x)、size()、next(x)、prev(x)。

宇宙[0,n)、0<=n<=100000000。next/prevはx自身を含む。見つからなければn/-1。検索閾値は宇宙外でも可。64の境界と空宇宙を扱い、シフト量64を使わない。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "static_query/fast_set.hpp"
int main() {
    cp::FastSet s(130); s.insert(3); s.insert(64); s.insert(129);
    std::cout<<s.next(4)<<" "<<s.prev(128)<<"\n";
}
```

出力: `64 64`。

## 計算量

構築/領域O(1+N/64)、更新/前後検索O(1+log₆₄(N+1))（64bit走査は定数）。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#fast_set) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/static_query/fast_set.cpp)
