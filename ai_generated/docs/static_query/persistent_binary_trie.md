---
title: Persistent Binary Trie
documentation_of: //static_query/persistent_binary_trie.hpp
---

# Persistent Binary Trie

uint64の各bitで分岐するTrieの根から葉までをコピーし、任意の版へ挿入・削除します。XOR後の順序統計も元の版を変えずに検索します。

## 使用例

```cpp
#include <iostream>
#include "static_query/persistent_binary_trie.hpp"
int main() {
    cp::PersistentBinaryTrie s; auto a=s.insert(0,7); auto b=s.insert(a,2);
    std::cout << *s.kth(a,0) << " " << *s.kth(b,0) << "\n";
}
```

出力: `7 2`。

## APIと使用条件

`PersistentBinaryTrie`、root=0は空、insert(root,x)は新root、erase(root,x)はoptional<int>、size(root)/count(root,x)、kth(root,k,xor_mask=0)。

全64bit、重複可。eraseは不在ならnullopt。kthはXOR後の値を返し、不正rankはnullopt。rootは同じインスタンスのもの、個数はuint64に収まること。

## 計算量

1操作64段（定数）、更新ごと65節点、検索は領域O(1)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#persistent_binary_trie) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
