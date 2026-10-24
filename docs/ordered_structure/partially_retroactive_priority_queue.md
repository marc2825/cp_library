---
title: Partially Retroactive Priority Queue
documentation_of: //ordered_structure/partially_retroactive_priority_queue.hpp
---

# Partially Retroactive Priority Queue（D-31）

事前登録した時刻上で、過去のinsert/delete-min操作を追加・削除し、現在の最小値と要素数を問い合わせます。履歴全体の再生は行いません。現在に残る挿入を重み0、既に消えた挿入を+1、delete-minを-1とし、prefix和0のbridgeを既存LazySegtreeで探索します。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#partially_retroactive_priority_queue)

## API・使用条件

`PartiallyRetroactivePriorityQueue<T,Time=long long,Compare=less<T>>(times,compare={})`。

- `insert(time,key)`, `insert_delete_min(time)` → 追加できたときtrue。
- `erase(time)` → 既存操作を消せたときtrue。
- `min()` → 現在の最優先値optional、`size()`, `empty()`。
- `contains_operation(time)`。

同じ時刻には1操作。時刻は構築時にsort/unique、未登録時刻はinvalid_argument。使用中時刻への追加・空時刻の削除・空キューへのdelete-minを生む編集はfalseで履歴を保持します。同値keyは挿入時刻の早い方から消えます。Compare=greaterなら最大優先。置換はerase後にinsertしますが、途中の履歴も有効である必要があります。メモリ確保・比較関数が投げる場合のtransaction保証はありません。

## 計算量・保存領域

登録時刻数mに対し構築 O(m log m)、操作の追加・削除・存在判定 O(log m)、現在のmin/size/emptyは O(1)、領域 O(m)。値・時刻のコピー・比較を O(1) と数えます。

## 使用例

[コンパイル可能な使用例](../../examples/ordered_structure/partially_retroactive_priority_queue.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../ordered_structure/partially_retroactive_priority_queue.hpp"
int main(){
    cp::PartiallyRetroactivePriorityQueue<int> queue({1,3,4,8});
    queue.insert(1,5); queue.insert(4,2); queue.insert_delete_min(8);
    std::cout<<*queue.min()<<' '<<queue.size()<<'\n';
    queue.insert(3,1);
    std::cout<<*queue.min()<<' '<<queue.size()<<'\n';
    queue.erase(8);
    std::cout<<*queue.min()<<' '<<queue.size()<<'\n';
}
```

```text
5 1
2 2
1 3
```

## 使用する問題

過去のキュー更新を編集するスケジューラ。現在時点だけを観測する部分retroactive APIです。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Demaine–Iacono–Langerman, Retroactive Data Structures](https://people.csail.mit.edu/edemaine/papers/Retroactive_TALG/paper.pdf)、Section 4.4のbridge補題。登録時刻を配列化し、既存LazySegtreeと独自の操作summary木で実現します。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
