---
title: Rollback Mo
documentation_of: //offline_query/rollback_mo.hpp
---

# Rollback Mo

削除が難しく、状態を履歴位置まで戻せる区間集計。

## API・使用条件

RollbackMo(n)、add_query(l,r)->ID、run(reset,add_left,add_right,snapshot,rollback,answer,block_size=0)。

snapshotはコピー可能なトークン、rollbackはその状態を完全に復元。長い問いは右端を単調に伸ばし、左側の一時追加だけ戻す。短い問いは空から右追加。run終了時resetで空、answer順は登録順とは限らない。空区間対応。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "offline_query/rollback_mo.hpp"
int main() {
    std::vector<int> a{2,3,5},history; int sum=0; cp::RollbackMo mo(3); mo.add_query(0,2); mo.add_query(1,3);
    auto add=[&](int p){history.push_back(p);sum+=a[p];};
    mo.run([&]{history.clear();sum=0;},add,add,[&]{return history.size();},
        [&](std::size_t token){while(history.size()>token){sum-=a[history.back()];history.pop_back();}},
        [&](int id){std::cout<<id<<":"<<sum<<" ";});
    std::cout<<"\n";
}
```

出力: `0:5 1:8`。

## 計算量

幅Bで追加操作O(QB+N²/B)、整列O(Q log(Q+1))、管理領域O(N/B+Q)。reset/rollback/answerの費用は別途。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#rollback_mo) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/offline_query/rollback_mo.cpp)
