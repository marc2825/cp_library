---
documentation_of: //dp_optimization/ida_star.hpp
---

# IDA* 反復閾値探索

[P-06の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#ida_star) / [ヘッダ](../../dp_optimization/ida_star.hpp)

`ida_star(start, neighbors, goal, heuristic, hash={}, equal={})` の状態・辺・結果は[A*](a_star.md)と共通です。非負long long費用、非負で許容的なheuristic、目標でh=0を前提とします。閾値を超えた最小のg+hへ次の閾値を上げ、経路上に既にある状態を除いて深さ優先探索します。全体の訪問済み集合による誤った刈り込みはしません。

再帰を使わない明示stackです。有限状態空間では単純パスが有限なので、費用0の循環や到達不能も終了します。最初の成立閾値で見つかった解が最適です。状態同一性が将来の遷移を決定すること、再訪による履歴依存効果がないことが必要です。

反復を含む展開回数をX、最大経路深さをd、最大分岐数をbとして、時間は遷移生成とXに比例、領域O(bd)。各stack frameが隣接辺列を保持するため単純なO(d)ではありません。最悪指数時間です。g+hの超過はoverflow_error、負費用/負hはinvalid_argument。結果のexpandedは全反復の累計です。

用途: メモリ制約下の最短パズル探索、許容下界を持つ組合せ状態。Floydとの比較、閉路、到達不能、深さ2万の鎖を検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/ida_star.hpp"
int main() {
    std::vector<std::vector<cp::StateSearchEdge<int>>> edges{
        {{1,5,0},{2,1,1}}, {{3,10,2}}, {{1,1,3}}, {}};
    std::vector<int> h{0,0,11,0};
    auto result = cp::ida_star(0, [&](int v) { return edges[v]; },
        [](int v) { return v == 3; }, [&](int v) { return h[v]; });
    std::cout << result->cost;
    for (int v : result->states) std::cout << ' ' << v;
    std::cout << '\n';
}
```

出力:

```text
12 0 2 1 3
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
