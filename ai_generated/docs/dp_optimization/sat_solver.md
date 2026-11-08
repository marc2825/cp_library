---
documentation_of: //dp_optimization/sat_solver.hpp
---

# 汎用CNF SAT Solver（DPLL）

[P-08の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#sat_solver) / [ヘッダ](../../dp_optimization/sat_solver.hpp)

`SatSolver(n)`、`add_clause(literals)`、`solve(assumptions={})` を公開します。literalはDIMACSと同じ **±(変数番号+1)**、0は拒否。結果はn個のboolの代入、UNSATならnulloptです。空節は矛盾、重複literalは除去、x∨¬xを含む節は恒真として省きます。n=0の空CNFは空代入でSATです。

2個のwatched literalを使う単位伝播と、出現数の多い未代入変数からのDPLL分岐です。明示decision stackで反対の値へバックトラックします。監視位置はバックトラックで戻す必要がなく、巻き戻した変数の値だけを未代入へ戻します。

solveはCNFの作業コピーを使い、assumptionsはその呼び出し限りです。solve後の節追加・再solveを許容しますが、学習節や前回探索木は再利用しません。CDCL、非年代順backjump、UNSAT証明/unsat coreは提供しません。n<=INT_MAX/2、literalの範囲を実行時検査します。

最悪時間は変数数に指数的。単位伝播は実際に走査した監視節・literal数に比例し、分岐変数選択はO(n)。領域は変数数+全literal数に比例します。用途: 3-SATなど一般CNF、スケジューリングの小〜中規模厳密制約。真理値表5,000比較、鳩の巣原理、2万単位節、仮定・再求解を検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/sat_solver.hpp"
int main() {
    cp::SatSolver sat(3);
    sat.add_clause({1,2}); sat.add_clause({-1,3});
    sat.add_clause({-2,3}); sat.add_clause({-3,2});
    std::cout << bool(sat.solve()) << ' ' << bool(sat.solve({-3})) << '\n';
}
```

出力:

```text
1 0
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
