---
title: Li Chao Tree DP
documentation_of: //dp_optimization/li_chao_tree.hpp
---

# Li Chao Tree DP

二乗距離のDPを直線の追加・最小値問い合わせへ変形します。

[実装](../../dp_optimization/li_chao_tree.hpp) / [操作できる図解](../../visualizations/math_a/index.html#li_chao_dp) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`cp::LiChaoTree<>`へ全ての問い合わせ座標を登録し、`add_line(a,b)`と`query(x)`を使用します。空の直線集合へのqueryはnullopt。 新しいCHT/木を複製せず、以下の使用例でDPへ組み込みます。

Li Chaoの係数・座標・評価の積と和はlong long。評価の超過はoverflow_error。未登録座標へのqueryはout_of_range。 DP側の平方・和も利用者が型内に収めます。例は非空列、dp[0]=0です。

## 原理と計算量

`dp[i]=min_{j<i}(dp[j]+(h[i]-h[j])^2+C)`を展開すると、傾き`-2*h[j]`、切片`dp[j]+h[j]^2`の直線の最小値に`h[i]^2+C`を足す形です。問い合わせてからiの直線を追加し、自分への遷移を防ぎます。

N個の直線と問い合わせでO(N log N)時間/O(N)領域。元のDPのO(N^2)と比較検証しています。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "dp_optimization/li_chao_tree.hpp"

int main() {
    std::vector<long long> h{10,30,20},dp(h.size());
    cp::LiChaoTree<> lines(h);
    for (int i=0;i<static_cast<int>(h.size());++i) {
        if (i) dp[i]=*lines.query(h[i])+h[i]*h[i]+10;
        lines.add_line(-2*h[i],dp[i]+h[i]*h[i]);
    }
    std::cout << dp.back() << "\n";
}
```

出力:

```text
110
```

[動く使用例](../../examples/dp_optimization/li_chao_dp.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [AtCoder DP Z - Frog 3](https://atcoder.jp/contests/dp/tasks/dp_z)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [KACTL LineContainer](https://github.com/kth-competitive-programming/kactl/blob/main/content/data-structures/LineContainer.h)、[既存Li Chao](li_chao_tree.md)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
