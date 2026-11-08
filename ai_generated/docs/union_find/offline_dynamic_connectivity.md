---
title: Offline Dynamic Connectivity（共通入口）
documentation_of: //union_find/offline_dynamic_connectivity.hpp
---

# Offline Dynamic Connectivity（共通入口）

3.6の実装を共有します。辺の有効期間を時間軸のセグメント木へ置き、Rollback DSUで各時刻の状態を復元します。

## 使用例

```cpp
#include <iostream>
#include "union_find/offline_dynamic_connectivity.hpp"
int main() {
    cp::OfflineDynamicConnectivity d(3); d.add_edge(0,1); d.add_query(0,2); d.add_edge(1,2); d.add_query(0,2);
    std::cout << d.solve()[0] << " " << d.solve()[1] << "\n";
}
```

出力: `0 1`。

## APIと使用条件

`OfflineDynamicConnectivity(n)`、add_edge(u,v)、remove_edge(u,v)は存在時true、add_query(u,v)は登録順ID、solve()はvector<bool>。

無向、多重辺・自己ループ可。同じ辺を複数追加した場合は同数の削除で非活性化。solveは再実行可。オンライン削除ではなく登録済みの操作列を一括処理。

## 計算量

Q操作に対してO(Q log Q log N)時間、O(N+Q log Q)領域。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

区間同士の同一視、オンライン辺追加、頂点群への一括加算、辺の削除を含む連結性などで使います。成分ラベルの直接更新、全辺を除いたBFSと比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#offline_dynamic_connectivity) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#dsu) / [図解の簡略化](../../visualizations/b_1_6/README.md)
