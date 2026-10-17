---
title: 周期的Grundy数列の検出
documentation_of: //game_theory/periodic_grundy.hpp
---

# 周期的Grundy数列の検出

観測列の反復候補を探し、有限の引き算ゲームでは十分な窓一致から未来の周期も証明します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/final_a/index.html#periodic_grundy) / [追加APIと検証](../final_a/README.md) / [一次資料](../final_a/references.md)

## API・条件

`detect_grundy_period(const vector<int>& values, size_t min_repetitions=3, size_t max_period=SIZE_MAX) -> optional<GrundyPeriod>`。非負の観測列の末尾について反復を検出します。GrundyPeriodはpreperiodとperiod。最小preperiod、次に最短periodを選びます。尾部の長さはmin_repetitions×period以上で、末尾に未完の1周期があっても可。空や候補なしはnullopt、max_period=0もnullopt。反復数は2以上、負の値は不可、違反はinvalid_argument。長さはINT_MAX以下。

**観測列だけの候補は将来の証明ではありません。** このAPIは未観測値を自動的に外挿しません。

`subtraction_grundy(vector<int> moves, size_t max_terms=100000) -> SubtractionGrundyResult`。固定された正の引き算手の集合についてg(0)からmax_terms個を計算します。手の重複は除去し、空集合は全0です。max_termsは1〜INT_MAX、非正の手はinvalid_argument。

結果はvalues、optional<GrundyPeriod> period、`value(uint64_t n)`。periodは最大手M個の同一窓が見つかった場合だけ設定します。証明がないときの未観測nはout_of_range。公開recordのvalues/periodを改変せず使ってください。

## Usage

```cpp
#include <iostream>
#include "game_theory/periodic_grundy.hpp"
int main() {
    const auto candidate = cp::detect_grundy_period({9, 0, 1, 0, 1, 0, 1});
    std::cout << candidate->preperiod << ' ' << candidate->period << '\n';
    const auto g = cp::subtraction_grundy({1, 3, 4}, 40);
    std::cout << g.period->preperiod << ' ' << g.period->period << '\n';
    std::cout << g.value(UINT64_MAX) << '\n';
}
```

出力:

```text
1 2
0 7
1
```

[コンパイルできる使用例](../../examples/game_theory/periodic_grundy.cpp)。

## 原理

逆順列のZ配列を使うと、各周期pに対する末尾の一致長を一括で求められます。一般の候補には必要な反復数を要求します。有限引き算ゲームのg(n)は直前M値だけで決まり、n>=Mでは手の合法性も変わりません。M連続値の窓が二箇所で一致すれば、その後は同じ遷移を続けるため周期が証明できます。手に巨大な値が含まれ、観測窓が足りなければ外挿を拒否します。

## 計算量

観測列の検出はO(N)時間・領域。引き算ゲームは手の重複除去にO(K log K)、列生成と証明にO(NK+N)時間・O(N+K)領域。valueはO(1)。見つからない場合でも指定したN項の計算で停止します。

## 使用する問題と検証

引き算集合が固定された通常ルールの石取りゲームで、巨大な山のGrundy値を求める用途。ゲーム依存の証明と外挿を扱うため、架空のPROBLEMは置かず、全手集合と素朴mex列・独立した状態窓の辞書で検証します。

`python3 verify/run_final_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した素朴解・境界と例外を確認します。[詳しい検証範囲](../final_a/README.md)。オンライン提出・ACは未確認です。
