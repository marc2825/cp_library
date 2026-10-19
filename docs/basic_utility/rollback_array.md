---
title: Rollback配列
documentation_of: //basic_utility/rollback_array.hpp
---

# Rollback配列（F-16）

固定長配列への変更をログに残し、過去のsnapshotまで巻き戻します。値だけでなく更新時刻や付随情報もTのstruct／pairへ入れれば一緒に復元できます。

## 使用例

```cpp
#include <iostream>
#include <utility>
#include "basic_utility/rollback_array.hpp"
int main() {
    cp::RollbackArray<std::pair<int, int>> state(2, {10, 0});
    auto before = state.snapshot();
    state.modify(0, [](auto& item) { item.first += 7; item.second = 1; });
    std::cout << state[0].first << " " << state[0].second << "\n";
    state.rollback(before);
    std::cout << state[0].first << " " << state[0].second << "\n";
}
```

出力:

```text
17 1
10 0
```

## API・条件

| API | 規約 |
|---|---|
| `RollbackArray(vector<T>)` / `(size_t n,const T& value=T{})` | 初期値・固定長を設定 |
| `operator[](i)` / `values()` | 読み取り専用。bool要素は値、他はconst参照 |
| `set(i,T value)` | 旧値を記録して代入。同値でもログを1件追加 |
| `modify(i,callback)` | 値のコピーをcallback(T&)で変更してから記録・反映 |
| `snapshot()` | 所有配列・履歴位置・枝を識別する不透明なSnapshot |
| `rollback(snapshot)` | 現在の履歴の祖先まで復元。snapshot自体は再利用可能 |
| `clear_history()` | 現在の値を保ってログを破棄し、既存snapshotを全無効化 |
| `size()/history_size()` | 配列長／現在のログ件数 |

Tはコピー構築可能、nothrow swap・nothrow破棄可能であることが必要です。boolにも対応します。配列のコピー／ムーブは禁止。範囲外添字はout_of_range。ログのコピーやcallbackが投げた場合、元の配列・ログは保持されます（callbackから配列自体を変更しないこと）。

別配列のsnapshot、巻き戻しで失われた枝のsnapshot、clear_history前のsnapshotはinvalid_argument。深さが同じになるまで再度更新しても、古い枝を有効とみなしません。snapshotは配列の寿命を延ばさず、破棄後は使用できません。ログ番号／epochがuint64上限に達するとlength_error。

外部に保存した集計値や別のオブジェクトは自動で復元しません。必要な情報をTにまとめるか、同じsnapshotから対象の配列要素をまとめて更新してください。resizeや任意の過去版を読み出す永続配列APIではありません。

TのコピーをO(1)とすると、set/modifyは償却O(1)、読み取りとsnapshotはO(1)、rollbackは取り消す件数kに対してO(k)、領域O(N+履歴数)。clear_historyはログの破棄件数に比例します。

## 使用する問題・検証

DFS分岐、分割統治、オフライン処理の状態復元。1万操作を配列丸ごとのコピーと比較し、付随情報、古い枝、別所有者、bool、失敗するコピー／callbackを検証します。

`make verify-foundation-tools` / `python3 verify/run_foundation_tools.py --sanitize`でDebug/Release、単独include、全カテゴリ混在、説明内使用例、独立比較を検証します。オンラインACは未確認です。

[追加範囲・検証](../foundation_tools/README.md) / [一次資料](../foundation_tools/references.md) / [カテゴリ索引](README.md) / [図解](../../visualizations/foundation_tools/index.html#rollback_array) / [図の範囲](../../visualizations/foundation_tools/README.md)
