# 3.1〜3.6 B項目

43掲載箇所を対象に、各カテゴリの公開ヘッダ・個別説明・動く使用例を揃えました。同じRange Treeの3.3／3.6を含む掲載単位の件数です。[実装予定リスト](../implementation_plan.md) / [操作できる図解](../../visualizations/b_1_6/index.html) / [一次資料](references.md)。

## 対応範囲

| カテゴリ | B掲載数 | 主な追加・共有 |
|---|---:|---|
| [3.1 基本ユーティリティ](../basic_utility/README.md) | 7 | Fractional Cascading、uint64整数根、Stern–Brocot、分数境界探索、平方分割、非単調分枝限定、既存RangeSet |
| [3.2 Union-Find](../union_find/README.md) | 5 | 区間同士のParallel、動的ラベル、成分・頂点加算、橋の増分管理、既存Offline Dynamic Connectivity |
| [3.3 セグメント木](../range_query/README.md) | 7 | 既存の疎なDynamic、Rollback、永続Lazy、Sortable、Range Tree、Fractional Cascading、可換2D Dual |
| [3.4 静的クエリ](../static_query/README.md) | 8 | 重み和Wavelet Matrix、Wavelet Tree、永続Trie、Priority Sum、動的bitset、rank/select、線形RMQ、Sqrt Tree |
| [3.5 平衡木・heap](../ordered_structure/README.md) | 9 | Splay、左傾Red-Black、再構築型Weight-Balanced、既存Lazy Treap、永続Treap、Skew、Leftist、既存Radix、遅延削除heap |
| [3.6 二次元・オフライン](../offline_query/README.md) | 7 | 矩形加算と和／点取得、Kd木、共通Range Tree、Hilbert Mo、種類数、オンライン点追加と矩形affine |

「Parallel」は同じ長さの二つの区間をoffset同士で併合する意味です。「動的DSU」は未知ラベルの追加で、辺のオンライン削除は含みません。非単調探索は正しい区間下界を必要とする有限整数区間の分枝限定法です。Sortableは一意な整数keyを扱います。2D Dualは可換な作用だけです。Dynamic Rectangle Affine Sumは点をオンライン追加でき、新点が過去の作用を受けない契約です。

Priority Sumの旧実装は公開フィールド互換のため保持し、標準名を明示したcp版を追加しました。旧Radix Heapはforward入口から共有し、浮動小数点encoderを新APIには公開しません。Sparse Dynamic、RangeSet、Offline Dynamic Connectivity、Lazy Balanced BSTも既存の実体を共有します。

## ローカル検証

```sh
python3 verify/run_b_1_6.py
python3 verify/run_b_1_6.py --sanitize
make verify-b-1-6
node verify/visualizations/b_1_6_model_test.js
node verify/visualizations/b_1_6_browser_test.mjs --browser /path/to/chromium
```

43公開入口の単独・重複includeをdebug/releaseで確認します。逆順include、全カテゴリall.hppを混在させる複数翻訳単位、43使用例と説明中のコードの一致、4本のC++比較検証と、4ジャッジ形式×24バッチの独立したPython解との比較を実施します。比較テストのCHECKはNDEBUGでも有効です。`--sanitize`はASan/UBSanを有効にして同じ使用例・比較検証を実行します。

- 基本・DSU: 小さい整数根と分数境界の全列挙、uint64最大値、600個の重複・空列catalog、平方分割の非可換文字列、54,000操作の成分ラベルと全頂点値・和、9,600グラフprefixで各辺を除いたBFS。
- Range・Static: Rollbackの履歴、任意の過去版からのLazy作用と検索の節点不増、std::sortと文字列順序、108,000静的問い合わせ、64bit境界をまたぐbit列、4,000Trie版、20,000Priority Sum操作。既定構築／==のないkey、20万要素のLinear RMQと13万要素のSqrt Treeで構築・queryの演算回数も確認。
- Ordered: 90,000多重集合操作をstd::multisetと比較、挿入・削除ごとの赤黒不変条件とweight条件、5,000永続分岐、30,000要素のheap併合とmove、遅延削除・単調Radix、既存Lazy Treapの組合せ。
- Spatial: 30,000矩形／最近点問い合わせ、24,000矩形更新を全セルと比較、52,000点追加／代入／affine操作、Hilbertの全格子の一意性と隣接性、40,000区間の種類数、LLONG_MIN/MAX、10,000点の偏った挿入。

空入力・重複・負数・空区間を含みます。正しい下界、モノイド／作用の法則、中間値の型内への収まり、各APIの前提は利用者の責任です。オンライン提出・ACは未確認です。

4ジャッジ形式は整数k乗根、Point Set Range Sort Range Composite、Static Rectangle Add Rectangle Sum、Static Range Count Distinctです。最大uint64、mod998244353、10^9付近の座標とkey、直接関数適用・矩形交差面積を使うPython解を含みます。

## 図解と実装の関係

図解は小入力の教材モデルです。[各モードの簡略化](../../visualizations/b_1_6/README.md)を明示し、入力変更・前後・再生で状態を確認できます。図解モデルの検証とC++の比較検証は別に実施します。

今回の検証結果: 通常・ASan/UBSan/LeakSanitizerのdebug/release、4形式×24バッチ、既存7系統の回帰が成功。図解モデルは4,112入力・131,777状態・8拒否条件、ブラウザは全42モードと390px表示が成功し、デスクトップ／モバイル画像を確認しました。関連ローカルリンク1,600件に欠落なし。socket／ptrace制限のあるブラウザ・サニタイザは、同じ検証を承認レビュー付きで制限外から実行しました。
