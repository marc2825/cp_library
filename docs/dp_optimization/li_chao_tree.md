---
title: Li Chao Tree
documentation_of: //dp_optimization/li_chao_tree.hpp
---

# Li Chao Tree

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#lichao) / [実装](../../dp_optimization/li_chao_tree.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "dp_optimization/li_chao_tree.hpp"
int main() {
    cp::LiChaoTree<> t({0, 1, 2, 3}); t.add_line(2, 1); t.add_line(-1, 5);
    t.add_segment(1, 3, 0, -7);
    std::cout << *t.query(0) << ' ' << *t.query(2) << ' ' << *t.query(3) << '\n';
}
```

[実行できる使用例](../../examples/dp_optimization/li_chao_tree.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`LiChaoTree<Minimize=true>(vector<long long> xs)`。`add_line(a,b)` はy=ax+bを全域、`add_segment(left,right,a,b)` はx∈[left,right)だけ追加。`query(x)->optional<long long>`、最大値は `<false>`。同じ傾き・交点・負数可。質問座標は構築時に全て登録します。未登録xはout_of_range、有効直線なしはnullopt。区間端点は未登録でもよい。空・逆向きの線分は無視。

評価a*xと+bがlong longを超えるとoverflow_error。例外までの追加が一部残る可能性があります。未評価の座標で超過がないことは追加時には検査しません。既存の [CHT](../../data_structure/ConvexHullTrick.hpp) は従来APIのまま保持しています。

## 計算量・原理

座標数Mとして構築O(M log M)、直線追加／質問O(log M)、線分追加O(log² M)、領域O(M)。座標の重複は除去。

区間中央で良い直線をそのノードに残し、交点の位置に応じて劣る直線を片側の子へ送ります。質問ではその座標への根から葉の候補を評価します。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/segment_add_get_min)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
