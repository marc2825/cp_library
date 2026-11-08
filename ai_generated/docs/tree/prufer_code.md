---
title: Prüfer Code
documentation_of: //tree/prufer_code.hpp
---

# Prüfer Code（T-02）

ラベル `0..n-1` の木を長さ `n-2` の列と相互変換します。各段階で最小ラベルの葉を取り除き、隣接頂点を出力します。復号時は符号内の出現回数+1を次数として、同じ規則で葉を選びます。

## API・条件

- `prufer_encode(g) -> vector<int>`：非空・無向・連結な木。辺の重みは無視します。木の検査は整数のトポロジーを作り、既存 `RootedTree` を共有するため、不要な重みの距離加算はしません。
- `prufer_decode(n, code) -> Graph<int>`：頂点数を明示します。`n=1` と `n=2` の符号は両方とも空。復号結果は単位重み、辺IDは復号時の追加順です。
- `n<=0`、長さが `max(0,n-2)` と異なる場合、`[0,n)` 外の符号、符号化対象の森・閉路・多重辺・自己ループ・有向グラフは `invalid_argument`。
- `n>=2` で長さとラベル範囲を満たす列は必ず木に復号できます。元の辺IDや隣接順は保存しません。

## 計算量・用途

最小葉を優先度キューで管理し、符号化・復号ともに O(n log n) 時間、O(n) 空間。ラベル付き木の列挙、木の全件テスト、Cayleyの公式の構成に使います。線形時間版の小さな定数より、APIと入力検査の明確さを選んでいます。

## 検証

n=2..7の全符号を復号し、独立した最小葉の直接走査と照合。相異なる符号が同じ辺集合を返さないことも確認します。n=1/2、不正入力、20万頂点の鎖の往復を検査しています。

## 使用例

```cpp
#include <iostream>
#include "../../tree/prufer_code.hpp"
int main() {
    auto tree = cp::prufer_decode(5, {3, 3, 1});
    auto code = cp::prufer_encode(tree);
    for (std::size_t i = 0; i < code.size(); ++i)
        std::cout << code[i] << (i + 1 == code.size() ? '\n' : ' ');
    std::cout << cp::prufer_decode(1, {}).size() << ' '
              << cp::prufer_decode(2, {}).edge_count() << '\n';
}
```

出力:

```text
3 3 1
1 1
```

[操作できる図解](../../visualizations/tree_extensions/index.html#prufer_code) / [検証範囲・コマンド](../tree_extensions/README.md) / [一次資料と設計判断](../tree_extensions/references.md) / [木カテゴリ](README.md)
