---
documentation_of: //dp_optimization/myers_edit_distance.hpp
---

# Myers Bit-parallel Edit Distance

[P-03の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#myers_edit_distance) / [ヘッダ](../../dp_optimization/myers_edit_distance.hpp)

`myers_edit_distance(string_view a, string_view b)` は列全体を一致させるLevenshtein距離をsize_tで返します。挿入・削除・置換は各1、一致は0。転置操作や任意の重み、部分文字列との最良一致は対象外です。256種類のバイト列で、NULも扱います。

垂直方向の差分±1をpositive/negative bit列で持ち、64bitのblock間に水平差分-1/0/+1を渡します。最上段の差分を+1にしてglobal alignmentとし、最後のblockは実際の最終bitで距離を更新します。入力長を64に制限しません。

n=max長、m=min長、B=ceil(m/64)。時間O(nB+m+256B)、領域O(256B)。空列の距離は他方の長さ。編集操作列の復元は行わず、既存の[通常編集距離](edit_distance.md)と用途を分けます。用途: 長い文字列の単位コスト比較、DNAなどのバイトalphabet。通常DP、word境界、長さ10万の不均衡な列で検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/myers_edit_distance.hpp"
int main() {
    std::cout << cp::myers_edit_distance("kitten", "sitting") << '\n';
}
```

出力:

```text
3
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
