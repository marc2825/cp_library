---
title: 汎用Undo Manager・スコア検算
documentation_of: //heuristic_search/undo_manager.hpp
---

# E-01 汎用Undo Manager・スコア検算

[一覧](README.md) / [ヘッダ](../../heuristic_search/undo_manager.hpp) / [実行例](../../examples/search_parsing_extensions/undo_manager.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#undo_manager) / [出典](references.md)

変更前の値や逆操作を記録し、履歴の末尾から逆順に戻す。スコア差分は全再計算と照合する。

## APIと条件

snapshot・rollback・transaction・perform・set、audit_score。

undoはnoexcept。参照先は履歴より長生きし、performの復元関数は途中まで失敗したforwardも元へ戻す。transactionの例外で複合変更を戻す。managerのコピー/moveは禁止。破棄は履歴を捨て、現在値を保持。

snapshot()は所有manager・履歴深さ・分岐識別子を保存します。別manager、rollback後に作り直した枝、clear_history()より前のsnapshotはinvalid_argument。transaction(F)はF(manager)の複合変更を例外時にまとめて戻し、入れ子を許します。transactionより前の履歴を消す操作は拒否します。成功時は履歴を残します。

set(target,value)は変更前の値をコピーし、noexcept swapで変更します。perform(forward,backward)は履歴を確保してからforwardを呼び、forwardが途中で投げてもbackwardで復元します。forward内の再帰的な履歴変更はlogic_error。backwardは履歴managerを変更してはいけません。setの参照先の再配置にも注意してください。clear_history()とデストラクタは値を戻しません。

audit_score(state,incremental,evaluate,tolerance)の返り値はincremental/recomputed/consistent。整数は完全一致、浮動数はnumerical::ErrorTolerance（絶対・相対誤差）で照合し、非有限scoreを拒否します。

## 計算量・用途

1変更はcallbackと値コピー以外で償却O(1)、rollbackは戻す件数に比例、履歴O(h)。auditは再評価の費用。

問題例・用途: 複合近傍の棄却、バックトラック中の履歴、差分scoreの開発時検算。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "heuristic_search/undo_manager.hpp"
#include <iostream>
#include <numeric>
using namespace cp;
int main() {
    std::vector<int> values{1, 2};
    UndoManager history;
    auto before = history.snapshot();
    history.transaction([&](auto &log) {
        log.set(values[0], 3);
        log.set(values[1], 4);
    });
    auto score = [](const auto &s) { return std::accumulate(s.begin(), s.end(), 0); };
    std::cout << score(values) << " " << audit_score(values, 7, score).consistent << " ";
    history.rollback(before);
    std::cout << score(values) << "\n";
}
```

出力:

```text
7 1 3
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
