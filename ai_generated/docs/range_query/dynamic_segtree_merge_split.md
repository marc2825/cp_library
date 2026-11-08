---
title: Dynamic Segtree merge/split
documentation_of: //range_query/dynamic_segtree.hpp
---

# Dynamic Segtree merge/split（D-07）

既存 DynamicSegtree に、座標域を保持する split と、重複点の結合規約を指定できる merge を追加しました。コピー後に別々の更新を行えます。

## API・使用条件

既存の `DynamicSegtree<S,op,e>(n)`、set/get/prod/all_prod/size/node_count は維持します。`split_copy(p)` は [0,p) と [p,n) の2本を返し、`split(p)` は自身を左側にして右側を返します。いずれも長さ n の同じ座標域です。右側の座標を詰め直しません。

`merged(other,combine=op)` は新しい木を返し、両入力を保持します。`merge(other,combine=op)` は自身を結果にして other を空にします。他に保存したコピーは保持します。同一オブジェクトを消費する merge と、異なる n の結合は invalid_argument。`a.merged(a)` は利用可能です。

同じ座標の**明示的に set した点**が両方にあるときだけ combine(自分の値,other の値) を呼びます。片側だけの点はそのまま移します。e() を set した点も存在する点です。既定は op ですが、上書きなど別の規約も指定できます。combine に結合律は不要、区間集約 op にはモノイド則が必要です。非可換なら座標の昇順を守ります。

不変ノードを shared_ptr で所有し、コピーと split の共有部分を自動管理します。node_count はその木から到達するノード数で、全版の通算確保数ではありません。最後の所有者が消えたノードを回収します。set/merge/split の例外時は元の木を保持します（callback の外部副作用は利用者の責任）。n は 0〜LLONG_MAX。

C++17、namespace cp、0始まり・半開区間。添字・範囲の前提は debug assert で検査します。比較・コピー・集約演算を O(1) として計算量を示します。整数の途中値は使用型に収まることが必要です。

## Usage

```cpp
#include <iostream>
#include "range_query/dynamic_segtree.hpp"
long long sum(long long a,long long b) { return a+b; }
long long zero() { return 0; }
int main() {
    cp::DynamicSegtree<long long,sum,zero> a(10);
    a.set(2,5); a.set(8,7);
    auto saved=a;
    auto right=a.split(5);
    a.set(2,9);
    a.merge(right);
    std::cout << saved.all_prod() << " " << a.all_prod() << " " << right.all_prod() << "\n";
    std::cout << a.merged(saved).get(2) << "\n";
}
```

```text
12 16 0
14
```

## 計算量

コピー O(1)、set/prod/split は O(log(n+1))、set/split の新規領域も O(log(n+1))。merged/merge は両方にノードがある交差部分を H として O(H+1) 時間・新規領域。片側が空の部分木は O(1) 共有。get も O(log(n+1))、all_prod/node_count O(1)。clear/破棄/代入は回収する非共有ノード数に比例する場合があります。

## 問題例・検証・参考資料

例題: 座標集合の分割・併合、部分木ごとの動的頻度表。6,000 の版分岐を map と文字列連結で比較し、重複点・座標保持・空域・LLONG_MAX・自己消費の拒否・例外時保持を検証。旧 DynamicSegtree のテストも回帰実行します。

[共通の検証入口・対応範囲](../data_structure_extensions/README.md) / [一次資料と設計判断](../data_structure_extensions/references.md)。オンライン AC は未確認です。

[操作できる図解](../../visualizations/data_structure_extensions/index.html#dynamic_segtree_merge_split) / [図解の範囲](../../visualizations/data_structure_extensions/README.md) / [動く例](../../examples/range_query/dynamic_segtree_merge_split.cpp) / [カテゴリ索引](README.md)
