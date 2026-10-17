---
title: 永続Stack
documentation_of: //ordered_structure/persistent_stack.hpp
---

# 永続Stack

任意の版からpush/popして旧版も保持する。

## API・使用条件

PersistentStack<T>、empty_root()->0、push(root,x)->root、pop(root)->root、top(root)->T、size(root)、empty(root)。

top/popは非空版のみ。ハンドルは同一オブジェクト由来、作成数はINT_MAX未満。Tはコピー可能、既定構築は不要。値は参照ではなくコピーを返す。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "ordered_structure/persistent_stack.hpp"
int main() {
    cp::PersistentStack<int> s; int a=s.push(0,4),b=s.push(a,9);
    std::cout<<s.top(a)<<" "<<s.top(b)<<" "<<s.top(s.pop(b))<<"\n";
}
```

出力: `4 9 4`。

## 計算量

各操作O(1)、push M回でO(M)領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#persistent_stack) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/ordered_structure/persistent_stack.cpp)
