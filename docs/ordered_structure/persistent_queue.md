---
title: 永続Queue
documentation_of: //ordered_structure/persistent_queue.hpp
---

# 永続Queue

任意の版からFIFO操作して分岐を保つ。二スタック反転の分岐時の再計算を避ける。

## API・使用条件

PersistentQueue<T>、empty_root()->0、push(root,x)->root、pop(root)->root、front/back(root)->T、size/empty(root)。

永続連結リストの末尾と論理長から、祖先ダブリングで先頭を求める。front/back/popは非空版のみ。ハンドルは同一オブジェクト由来、ノード/版/長さはINT_MAX未満。Tはコピー可能、既定構築不要。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "ordered_structure/persistent_queue.hpp"
int main() {
    cp::PersistentQueue<int> q; int a=q.push(0,4),b=q.push(a,9),c=q.pop(b);
    std::cout<<q.front(a)<<" "<<q.front(c)<<"\n";
}
```

出力: `4 9`。

## 計算量

push/frontは最悪O(log(M+1))、pop/back/sizeはO(1)。push M回＋pop P回でO(M log(M+1)+P)領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#persistent_queue) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/ordered_structure/persistent_queue.cpp)
