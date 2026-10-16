# 3.4 静的クエリ・順序統計・集合管理

今回の S 項目は、変更しない配列への問い合わせと、列の走査・移動する区間の管理です。C++17、`namespace cp`、0 始まりの添字を使います。必要なヘッダだけ include でき、まとめる場合は `static_query/all.hpp` を指定します。

| やりたいこと | 実装・API | 詳細 | 図解 |
|---|---|---|---|
| 静的な min/max/gcd など | [`SparseTable<S,op,e>`](../../static_query/sparse_table.hpp) | [説明・usage](sparse_table.md) | [2 区間の重なり](../../visualizations/static_query/index.html#sparse) |
| 直前・直後の小さい／大きい要素 | [`MonotonicStack<T,Compare>` と関数](../../static_query/monotonic_stack.hpp) | [説明・usage](monotonic_stack.md) | [候補を取り除く](../../visualizations/static_query/index.html#stack) |
| FIFO の最小／最大値と添字 | [`MonotonicQueue<T,Compare>`](../../static_query/monotonic_queue.hpp) | [説明・usage](monotonic_queue.md) | [候補と論理キュー](../../visualizations/static_query/index.html#queue) |
| 全長 k の窓の最小値・argmin | [`sliding_window_minimum/argmin`](../../static_query/sliding_window_minimum.hpp) | [説明・usage](sliding_window_minimum.md) | [窓を動かす](../../visualizations/static_query/index.html#window) |
| FIFO に対する任意のモノイドの集約 | [`SWAG<S,op,e>`](../../static_query/swag.hpp) | [説明・usage](swag.md) | [2 スタックの向き](../../visualizations/static_query/index.html#swag) |

Sparse Table は区間の両端が任意でも使えますが、O(1) 取得には冪等性が必要です。単調キュー・SWAG は右端への追加と左端からの削除を繰り返す場合に使います。値の更新や任意区間の和には [3.3 のセグ木・Fenwick Tree](../range_query/README.md) を選べます。

## 共通の契約

- 区間は半開区間 `[l,r)`。添字が見つからない最近傍検索は -1 を返します。
- Sparse Table と SWAG の空の積は e()。空キューの pop/best/front/back、空スタックの top は呼べません。前提はデバッグ時に assert で検査します。
- 比較器は厳密弱順序です。`std::less<T>` が小さいもの優先、`std::greater<T>` が大きいもの優先。等値は比較器で両方向とも false の場合です。
- 単調キューの同値の最良値は最も古い（左の）添字を返します。スタックの strict は「厳密な小なり／大なり」を選ぶための指定です。
- 演算・比較・コピーが O(1) の場合の計算量です。文字列連結などでは時間と記憶量が増えます。
- 返す参照や candidates の要素は読み取り用です。更新・clear 後には参照を保持しないでください。
- 通算 push 数（clear でリセット）は単調スタック・キューで `INT_MAX` 以下。SWAG の同時要素数も `INT_MAX` 以下です。

## 検証

```sh
python3 verify/run_static_query.py
python3 verify/run_static_query.py --sanitize
node verify/visualizations/static_query_model_test.js
```

6 ヘッダの単独／重複 include（debug/release）、逆順 include、複数翻訳単位、3 使用例、5 ジャッジ用プログラムを確認します。長さ 0〜8 の三値配列 9,841 個を全探索し、全区間の RMQ、最近傍、全窓を素朴解と比較します。さらに 240 ランダム FIFO 履歴・120,000 操作、20 万要素、比較・集約演算回数の線形上限を検証します。

重複値、負数、空入力、k=1 / n / n+1、カスタム比較器、デフォルトコンストラクタのない型、非可換な冪等演算、文字列とアフィン合成を含みます。オンライン提出による AC を表すものではありません。

[動く使用例](../../examples/static_query/) / [ジャッジ用プログラム](../../verify/static_query/) / [参考資料・設計判断](references.md) / [すべての図解](../../visualizations/index.html)

## 高度なアルゴリズムの追加

- [Wavelet Matrix](wavelet_matrix.md) / [操作できる図解](../../visualizations/advanced/index.html#wavelet)

追加分の検証は `python3 verify/run_advanced.py`（`--sanitize`対応）。[全追加項目と既存項目の対応表](../advanced/README.md)。
