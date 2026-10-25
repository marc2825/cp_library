# 未実装・追加仕様の統合一覧

更新日: 2026-10-25。初回照合基準はコミット `9067220` と、この依頼の本文・添付2資料・元の3.1〜3.25のC項目。C++版を対象とし、同じ機能・別名・カテゴリ間の重複を1行へ統合しました。

[実装計画](implementation_plan.md) / [引継書](../HANDOFF.md) / [ドキュメント索引](README.md)

これは初回監査と追加実装の進捗を追跡する一覧です。2026-10-19の「未実装のものを順に」の依頼に従い、先頭のF-01〜F-17を整備しました。[F-01〜F-07](basic_utility/foundation.md) / [F-08〜F-10の対応範囲・検証](foundation_structures/README.md)。[F-11の対応範囲・検証](basic_utility/big_integer.md)。[F-12〜F-17の対応範囲・検証](foundation_tools/README.md)。[V-01〜V-04の開発・検証基盤](development/README.md)も整備しました。[V-05〜V-07の実行・観察ツール](development_runtime/README.md)も整備しました。[V-08〜V-10の出典・検証台帳と共通debug mode](development_records/README.md)も整備しました。[U-01〜U-11・U-20〜U-22の14項目](utility_search/README.md)も整備しました。[D-01（SWAG Deque）](static_query/swag_deque.md)も整備しました。[D-02〜D-10](data_structure_extensions/README.md)も整備しました。[D-20〜D-35の16項目](data_structure_advanced/README.md)も整備しました。今回の指定に従い[T全7項目](tree_extensions/README.md)も整備しました。D-11〜D-19の行はなく、[G-01〜G-15](graph_extensions/README.md)も整備しました。掲載順の未着手先頭はG-16です。初回監査では既存コードと公開API・説明・検証の範囲を確認しました。添付にあるLibrary Checkerの新着区分・他ライブラリの収録状況全体は再監査していません。T追加分の出典と動的木2問題の入出力仕様は[一次資料](tree_extensions/references.md)で別途確認しました。

資料1は「1. 追加対象の区分」で始まる添付、資料2は「調査範囲と凡例」で始まる添付です。本文と2資料の候補を含め、元Cは末尾の全件対応表で追跡します。既存S全体の再整備やPythonへの全移植は別の計画で、この一覧の件数に含めません。

## 件数と判定基準

| 区分 | 件数 | 意味 |
|---|---:|---|
| 未実装 | 114 | 該当する公開機能を確認できない。 |
| 一部対応 | 19 | 基礎/関連機能はあり、記載した追加仕様・汎用化・性能が不足。 |
| 整備待ち | 5 | 旧実装あり。再実装より先に新API/単独include/説明/検証を整える。 |
| 要仕様化 | 18 | 「特殊な」「高度な」等で範囲が決まらず、完成/未実装を一律に断定できない。 |
| 残作業の合計 | 156 | 下表の未完了行。元リストの掲載数とは異なる。 |
| 追加整備済み | 89 | F-01〜F-17、V-01〜V-10、U-01〜U-11・U-20〜U-22、D-01〜D-10・D-20〜D-35、T-01〜T-03・T-20〜T-23、G-01〜G-15。完了行もIDを保持し追跡する。 |
| 追跡対象の合計 | 245 | 残作業156＋追加整備済み89。 |
| 既存対応で除外 | 27 | 後掲の参照表。残作業には重複計上しない。 |

「旧Cはすべて未実装」という前提は採用しません。たとえば元3.15 CのP-recursive数列は3.18 Bで既知漸化式のprefix生成が実装済みです。一方、Holonomicの推定・閉包操作などは別の残作業です。O(n³)の特性多項式・既存の中間積・基本rank/select等も、基礎機能と高度化を分けています。

複数のAPIを1行にまとめたものは共通基盤を持つ作業単位です。例：Resultant/Discriminant、Matroid Union/Partition、線分スイープ/Arrangement。別用途のRange Mex・Range Majority・Range Mode、木の中心・重心、mod 2^64・GF(2^64)は統合しません。追加資料のS/A/B/C案と元リストのランクは混ぜず、出所欄のC3.xだけが元Cを意味します。

## 着手順の候補

1. 基盤型・FastIO整備・共通安全演算・verify CI/stress tester。
2. 尺取り・動的MEX・半分全列挙・Knapsack復元。Segtree境界探索やLIS/LCS復元は既存を使う。
3. 区間辺グラフ・二部被覆の復元・長方形和集合。
4. 素数冪/CRT二項係数・任意mod行列式・SWAG Deque・Gomory–Hu。
5. 数値解析・LP・専門代数・圧縮索引と元C。2026-10-19から一覧掲載順で着手し、F-01〜F-17は整備済み。V-01〜V-04も整備済み。V-05〜V-07も整備済み。V-08〜V-10も整備済み。U-01〜U-11・U-20〜U-22も整備済み。D-01も整備済み。D-02〜D-10・D-20〜D-35も整備済み。今回の指定でT全7項目も整備済み。G-01〜G-15も整備済み。掲載順の未着手先頭はG-16です。

## 基盤型・入出力・共通基盤

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="f-01"></a>F-01 | static_modint | 実装あり | 正規化・四則演算・uint64指数・optional逆元・合成数法を公開。uint32全域の法。 [実装](../basic_utility/static_modint.hpp) / [説明・検証](basic_utility/static_modint.md)。 | 追加 |
| <a id="f-02"></a>F-02 | dynamic_modint | 実装あり | 実行時uint32法・複数ID・法変更時の世代検査・逆元条件を実装。 [実装](../basic_utility/dynamic_modint.hpp) / [説明・検証](basic_utility/dynamic_modint.md)。 | 追加 |
| <a id="f-03"></a>F-03 | Barrett Reduction | 実装あり | 正のuint32法についてuint64全域の剰余・uint32積を公開。 [実装](../basic_utility/barrett_reduction.hpp) / [説明・検証](basic_utility/barrett_reduction.md)。 | 追加 |
| <a id="f-04"></a>F-04 | Montgomery・64bit高性能modint | 実装あり | 奇数法1以上2^63未満。実行時REDCと固定法64bit modint、符号付き／なし128bit入力に対応。 [実装](../basic_utility/montgomery_modint.hpp) / [説明・検証](basic_utility/montgomery_modint.md)。 | 追加, C3.13 |
| <a id="f-05"></a>F-05 | 2^61−1 modint | 実装あり | 2^61−1専用の高速積・四則演算と部分文字列ハッシュへの接続例。 [実装](../basic_utility/modint61.hpp) / [説明・検証](basic_utility/modint61.md)。 | 追加 |
| <a id="f-06"></a>F-06 | FastIOの共通化・単独ヘッダ化 | 実装あり | 独立したFILE*・バッファの共通FastScanner/Printer。128bit・浮動小数点・EOF・極値・不正入力を検証。旧2入口は保持し移行を説明。 [実装](../basic_utility/fast_io.hpp) / [説明・検証](basic_utility/fast_io.md)。 | 追加 |
| <a id="f-07"></a>F-07 | 安全な整数演算の共通API | 実装あり | 検査付き／飽和加減乗算、非負の上限判定・番兵加算、符号付きfloor/ceil除算。128bit対応。 [実装](../basic_utility/safe_integer.hpp) / [説明・検証](basic_utility/safe_integer.md)。 | 追加 |
| <a id="f-08"></a>F-08 | 乱数付きHash Map・複合キーハッシュ | 実装あり | 線形探索のopen addressing、削除印・再構築、整数/pair/tuple/列、seed注入と再現。 [HashMap](../basic_utility/hash_map.hpp) / [SeededHash](../basic_utility/seeded_hash.hpp) / [説明・検証](basic_utility/hash_map.md)。 | 追加 |
| <a id="f-09"></a>F-09 | CSR Graph・静的隣接リスト | 実装あり | 辺ID・逆向きarc・入力順・自己ループ・有向/無向、Graphへの往復変換。 [実装](../graph/csr_graph.hpp) / [説明・検証](graph_basic/csr_graph.md)。 | 追加 |
| <a id="f-10"></a>F-10 | Monoid / ActedMonoidの共通抽象・カタログ | 実装あり | 既存sum/add/assign/affineを共有し、min/max＋個数・最大部分配列・行列・関数・一次分数・hash・等差/多項式加算を補完。 [カタログ](range_query/monoid_catalog.md) / [多項式作用](range_query/polynomial_add_sum.md)。 | 追加 |
| <a id="f-11"></a>F-11 | 多倍長整数 | 実装あり | Boost cpp_intラッパーBigInt、加減乗除・剰余・GCD/LCM・厳密な10進/16進I/O・範囲検査付き整数変換。入力1万桁・積2万桁を独立比較し性能計測を記録。 [実装](../basic_utility/big_integer.hpp) / [説明・検証](basic_utility/big_integer.md)。 | 追加 |
| <a id="f-12"></a>F-12 | 多倍長有理数 | 実装あり | BigInt上の分母正・既約分数、四則演算・比較・整数／n/d入出力、Matrix等へ接続。 [実装](../basic_utility/big_rational.hpp) / [説明・検証](basic_utility/big_rational.md)。 | 追加 |
| <a id="f-13"></a>F-13 | メモリプール・Arena Allocator | 実装あり | 型付きブロックArena。追加時の安定ID/参照、整列・逆順破棄・clear、永続木の経路コピー例を検証。 [実装](../basic_utility/arena.hpp) / [説明・検証](basic_utility/arena.md)。 | 追加 |
| <a id="f-14"></a>F-14 | ビット演算ユーティリティ | 実装あり | uint64 topbit/lowbit/popcount、0と端の規約、Gosperによる固定popcountの遅延列挙。 [実装](../basic_utility/bit_operations.hpp) / [説明・検証](basic_utility/bit_operations.md)。 | 追加 |
| <a id="f-15"></a>F-15 | グリッドユーティリティの共通化 | 実装あり | 4/8近傍、矩形・境界、時計回り回転・転置。既存Grid BFSの方向・境界処理を共有。 [実装](../basic_utility/grid.hpp) / [説明・検証](basic_utility/grid.md)。 | 追加 |
| <a id="f-16"></a>F-16 | 汎用Rollback配列 | 実装あり | 固定長配列のset/modify、枝を識別するsnapshot/rollback、値と付随情報、履歴破棄。 [実装](../basic_utility/rollback_array.hpp) / [説明・検証](basic_utility/rollback_array.md)。 | 追加 |
| <a id="f-17"></a>F-17 | 基数ソート | 実装あり | 8〜128bitの符号付き/なしキー、安定LSD、projection・レコード整列、例外時の元配列保持。 [実装](../basic_utility/radix_sort.hpp) / [説明・検証](basic_utility/radix_sort.md)。 | 追加 |

## 開発・検証基盤

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="v-01"></a>V-01 | verify-helper / competitive-verifier構成・CI | 実装あり | competitive-verifier 4.1.2と依存版固定、GitHub Actions、3問題＋standalone、文書artifact生成。実CLIのローカル検証と依存解決済み。GitHub実行・オンライン全ケースは未確認。 [説明・検証](development/ci.md)。 | 追加 |
| <a id="v-02"></a>V-02 | 汎用stress tester | 実装あり | argvで生成器・高速解・愚直解を実行。seed/入出力/終了状態保存、プロセスごとのtimeout、保存入力でreplay。 [説明・検証](development/stress.md)。 | 追加 |
| <a id="v-03"></a>V-03 | 共有ランダムデータ生成器 | 実装あり | 固定SplitMix64、木・森・成分数指定/多重グラフ・DAG・凸な単純多角形のPython開発用共有API。生成分布と条件を明記。 [説明・検証](development/random_data.md)。 | 追加 |
| <a id="v-04"></a>V-04 | 提出用バンドラ | 実装あり | リテラルlocal include展開、guardと条件付き依存、重複排除、依存一覧、展開後compile check。5実プログラムの元/展開後比較。 [説明・検証](development/bundle.md)。 | 追加 |
| <a id="v-05"></a>V-05 | デバッグ出力ライブラリ | 実装あり | コンテナ・tuple・文字列・128bit・optional/variant・任意型formatter。CP_DEBUGは既定無効、提出時に引数評価ごと除去。 [説明・検証](basic_utility/debug.md)。 | 追加 |
| <a id="v-06"></a>V-06 | インタラクティブ問題用ローカルジャッジ | 実装あり | POSIX双方向pipe、部分転送とEOF、wall/idle timeout、出力量上限、binary通信ログ・stderr・終了状態保存。 [説明・検証](development_runtime/interactive.md)。 | 追加 |
| <a id="v-07"></a>V-07 | AHCの複数seed並列実行・スコア集計 | 実装あり | 外部生成器/solver/scorerを複数seed・設定で並列実行。同じ入力、全seed成功設定だけの最良比較、保存/replay、timeout・キャンセル。 [説明・検証](development_runtime/benchmark.md)。 | 追加 |
| <a id="v-08"></a>V-08 | 出典・ライセンス・取り込み版の台帳 | 実装あり | 移植・旧帰属・外部依存8レコードの出典/版/ライセンス/変更内容とSHA-256を追跡。不明版・未確認ライセンスを明示し、コード変更や未登録帰属を検査。 [説明・検証](development_records/provenance.md)。 | 追加 |
| <a id="v-09"></a>V-09 | 機能と検証問題の対応・検証状況の自動集計 | 実装あり | 全ヘッダ・manifest・検証ソース・問題を自動集計。ソースfingerprint付き実行記録と提出AC証拠を分け、未verify/ローカルのみ/提出ACを検索可能なHTMLとJSONへ表示。 [説明・検証](development_records/verification.md)。 | 追加 |
| <a id="v-10"></a>V-10 | 標準ライブラリdebug modeの共通検証設定 | 実装あり | 既存の全runnerを選べる共通launcher。libstdc++ debug mode・NDEBUGとの独立・既存サニタイザ併用・2翻訳単位・無効イテレータの検出を検証。 [説明・検証](development_records/stdlib_debug.md)。 | 追加 |

## 基本ユーティリティ・列挙・探索（3.1）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="u-01"></a>U-01 | 尺取り法 / Two Pointers | 実装あり | 部分区間に閉じた条件のadd/remove callback、最大右端・最長区間・非空区間数・早期停止列挙。 [実装](../basic_utility/two_pointers.hpp) / [説明・検証](basic_utility/two_pointers.md)。 | 追加 |
| <a id="u-02"></a>U-02 | 最大部分配列和 / Kadane | 実装あり | 区間復元、全負値、空区間可否、辞書順の同点規則、整数の加算検査。 [実装](../basic_utility/maximum_subarray.hpp) / [説明・検証](basic_utility/maximum_subarray.md)。 | 追加 |
| <a id="u-03"></a>U-03 | 動的MEX | 実装あり | int64の重複・負値・挿入・1個削除、被覆区間を共有したMEX。 [実装](../basic_utility/dynamic_mex.hpp) / [説明・検証](basic_utility/dynamic_mex.md)。 | 追加 |
| <a id="u-04"></a>U-04 | 半分全列挙 / Meet-in-the-Middle | 実装あり | ソート済み部分和、128bit和、条件付き個数・最大和・採用mask復元。 [実装](../basic_utility/meet_in_the_middle.hpp) / [説明・検証](basic_utility/meet_in_the_middle.md)。 | 追加 |
| <a id="u-05"></a>U-05 | 順列の基本演算 | 実装あり | 合成方向を明示した合成・逆・サイクル・符号・負指数を含む冪。既存rank/unrankは共有。 [実装](../basic_utility/permutation.hpp) / [説明・検証](basic_utility/permutation.md)。 | 追加 |
| <a id="u-06"></a>U-06 | 黄金分割探索・フィボナッチ探索 | 実装あり | 有限閉区間の黄金分割、整数半開区間のFibonacci探索。単峰性・同点・評価回数を明示。 [実装](../basic_utility/unimodal_search.hpp) / [説明・検証](basic_utility/unimodal_search.md)。 | 追加 |
| <a id="u-07"></a>U-07 | 集合・多重集合ハッシュ | 実装あり | 集合XOR・多重集合和を2本、頻度・削除・明示seed・衝突条件。既存SeededHashを共有。 [実装](../basic_utility/collection_hash.hpp) / [説明・検証](basic_utility/collection_hash.md)。 | 追加 |
| <a id="u-08"></a>U-08 | Permutation Tree / Common Interval Decomposition | 実装あり | O(n log n)の強共通区間木、増加・減少・Prime、区間数。MIT出典を記録。 [実装](../basic_utility/permutation_tree.hpp) / [説明・検証](basic_utility/permutation_tree.md)。 | 追加 |
| <a id="u-09"></a>U-09 | 括弧列ユーティリティ | 実装あり | 正規判定・仮想根付き順序木との非再帰変換・BigIntの辞書順rank/unrank。 [実装](../basic_utility/parentheses.hpp) / [説明・検証](basic_utility/parentheses.md)。 | 追加 |
| <a id="u-10"></a>U-10 | 整数分割の列挙 | 実装あり | 非増加整数分割の反復列挙・早期停止とBigIntの分割数。 [実装](../basic_utility/integer_partitions.hpp) / [説明・検証](basic_utility/integer_partitions.md)。 | 追加 |
| <a id="u-11"></a>U-11 | 集合分割の列挙 | 実装あり | Restricted-growth列の辞書順列挙・ブロック変換・BigIntのBell数。 [実装](../basic_utility/set_partitions.hpp) / [説明・検証](basic_utility/set_partitions.md)。 | 追加 |
| <a id="u-20"></a>U-20 | 部分永続の探索補助 | 実装あり | 最新版だけの点代入配列・過去版参照・単調述語の最初の成立版検索と既存部分永続DSUアダプタを仕様化・実装。 [実装](../basic_utility/partial_persistent_search.hpp) / [説明・検証](basic_utility/partial_persistent_search.md)。 | C3.1 |
| <a id="u-21"></a>U-21 | Retroactiveな操作管理 | 実装あり | 事前登録したint64時刻上のMonoid操作の追加・置換・削除、任意時刻・区間の合成。既存Segtreeで各O(log m)。 [実装](../basic_utility/retroactive_operations.hpp) / [説明・検証](basic_utility/retroactive_operations.md)。D-31のPriority Queueは別項目。 | C3.1 |
| <a id="u-22"></a>U-22 | 特殊な整数列挙 | 実装あり | uint64の回文・数字制限・指定因子の積を昇順・重複なし・途中停止可能に列挙。大数の狭い区間とoverflowを処理。 [実装](../basic_utility/special_integer_enumeration.hpp) / [説明・検証](basic_utility/special_integer_enumeration.md)。 | C3.1 |

## データ構造・時間軸処理（3.2〜3.6）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="d-01"></a>D-01 | SWAG Deque | 実装あり | 両端push/popはならしO(1)、順序を保つ全体積・端の参照はO(1)。半分ずつの再構築、非可換・非可逆演算、線形演算回数を検証。 [実装](../static_query/swag_deque.hpp) / [説明・検証](static_query/swag_deque.md)。既存Queue版は保持。 | 追加 |
| <a id="d-02"></a>D-02 | Double-Ended Priority Queue | 実装あり | std::multiset共有の両端キュー。重複・比較順・コピーを検証。 [実装](../ordered_structure/double_ended_priority_queue.hpp) / [説明・検証](ordered_structure/double_ended_priority_queue.md)。 | 追加 |
| <a id="d-03"></a>D-03 | Static Range MEX | 実装あり | prefixごとの最終出現位置を永続化し、任意区間MEXをO(log n)。負数・巨大値・空区間を検証。 [実装](../static_query/static_range_mex.hpp) / [説明・検証](static_query/static_range_mex.md)。 | 追加 |
| <a id="d-04"></a>D-04 | Range Majority | 実装あり | 相殺候補と出現位置による頻度検証。過半数なしはnullopt。 [実装](../static_query/range_majority.hpp) / [説明・検証](static_query/range_majority.md)。 | 追加 |
| <a id="d-05"></a>D-05 | Point Set Range Frequency | 実装あり | オンラインの任意値への点代入と区間頻度をO(log² n)。標準区間のmapを更新。 [実装](../range_query/point_set_range_frequency.hpp) / [説明・検証](range_query/point_set_range_frequency.md)。 | 追加 |
| <a id="d-06"></a>D-06 | 公開のPersistent Leftist Heap | 実装あり | 汎用pool・版root・線形build・永続meld/push/pop。k_shortest_walksと共有し所有権を明記。 [実装](../ordered_structure/persistent_leftist_heap.hpp) / [説明・検証](ordered_structure/persistent_leftist_heap.md)。 | 追加 |
| <a id="d-07"></a>D-07 | 動的Segtreeの公開merge/split | 実装あり | 不変ノードを共有するsplit_copy/mergedと消費型split/merge。座標保持、重複点のcombine、旧版保持を検証。 [実装](../range_query/dynamic_segtree.hpp) / [説明・検証](range_query/dynamic_segtree_merge_split.md)。 | 追加 |
| <a id="d-08"></a>D-08 | 区間の標準ノード分解ユーティリティ | 実装あり | 完全二分木の標準ノードへ左から右にO(log n)分解。番号規約・padding・空域・巨大域を検証。 [実装](../range_query/segment_tree_cover.hpp) / [説明・検証](range_query/segment_tree_cover.md)。 | 追加 |
| <a id="d-09"></a>D-09 | 汎用Segment Tree over Time | 実装あり | 任意Stateのsnapshot/apply/rollbackを受ける時間軸セグ木。例外時も復元。OfflineDynamicConnectivityと共有。 [実装](../offline_query/segment_tree_over_time.hpp) / [説明・検証](offline_query/segment_tree_over_time.md)。 | 追加 |
| <a id="d-10"></a>D-10 | DSUの併合callback・付随情報のRollback | 実装あり | 通常/rollback DSUの併合直前callbackと任意Dataの復元ラッパー。借用・履歴・例外・分岐の契約を明記。 [実装](../union_find/rollback_disjoint_set_union_with_data.hpp) / [説明・検証](union_find/rollback_disjoint_set_union_with_data.md)。 | 追加 |
| <a id="d-20"></a>D-20 | 非可換群上のPotentialized DSU | 実装あり | 非可換群のop/inverseによる差分制約、経路圧縮・矛盾判定。 [実装](../union_find/group_potentialized_disjoint_set_union.hpp) / [説明・検証](union_find/group_potentialized_disjoint_set_union.md)。 | C3.2 |
| <a id="d-21"></a>D-21 | 一般的なFully Dynamic Connectivity | 実装あり | オンライン一般無向グラフ。既存ETTの全域森と小成分の代替辺走査、多重辺・自己ループ。木辺削除の最悪は線形。 [実装](../union_find/fully_dynamic_connectivity.hpp) / [説明・検証](union_find/fully_dynamic_connectivity.md)。 | C3.2 |
| <a id="d-22"></a>D-22 | Kinetic Segment Tree | 実装あり | 単調な整数時刻の線形値、交差certificate・点変更・区間最小。128bit評価。 [実装](../range_query/kinetic_segtree.hpp) / [説明・検証](range_query/kinetic_segtree.md)。 | C3.3 |
| <a id="d-23"></a>D-23 | RBST-based Segment Tree | 実装あり | 仕様: 動的列の点更新・区間積・反転・split/concat。サイズ比RBSTをD-32と共有。 [実装](../ordered_structure/rbst_segment_tree.hpp) / [説明・検証](ordered_structure/rbst_segment_tree.md)。 | C3.3 |
| <a id="d-24"></a>D-24 | 複雑な範囲変換をサポートするBeats | 実装あり | 仕様: 非負整数の区間floor除算・加算・代入とsum/min/max。min/maxの商・差分で枝刈り。 [実装](../range_query/division_segment_tree_beats.hpp) / [説明・検証](range_query/division_segment_tree_beats.md)。 | C3.3 |
| <a id="d-25"></a>D-25 | Static Range Mode | 実装あり | ブロック区間の最頻候補と出現位置による再検証。同率はCompare順。 [実装](../static_query/static_range_mode.hpp) / [説明・検証](static_query/static_range_mode.md)。 | C3.4 |
| <a id="d-26"></a>D-26 | Static Range LIS | 実装あり | CC0のSeaweed / unit-Mongeを固定版で導入。重複・Compare・空入力、構築O(n log² n)、query O(log n)。 [実装](../static_query/static_range_lis.hpp) / [説明・検証](static_query/static_range_lis.md)。 | C3.4 |
| <a id="d-27"></a>D-27 | 静的転倒数クエリ | 実装あり | 既存Mo/Fenwickを公開関数化。旧ジャッジラッパーも共通APIに接続。 [実装](../offline_query/static_range_inversions.hpp) / [説明・検証](offline_query/static_range_inversions.md)。 | C3.4 |
| <a id="d-28"></a>D-28 | 動的Wavelet Matrix | 実装あり | uint64の挿入・削除・点代入とrank/select・順序統計。動的bit列を共有。 [実装](../range_query/dynamic_wavelet_matrix.hpp) / [説明・検証](range_query/dynamic_wavelet_matrix.md)。 | C3.4 |
| <a id="d-29"></a>D-29 | 特殊なRank/Select構造 | 実装あり | 仕様: 挿入・削除可能なrank/selectビット列。implicit treap、非圧縮O(n) words、削除ノード再利用。 [実装](../ordered_structure/dynamic_bit_vector.hpp) / [説明・検証](ordered_structure/dynamic_bit_vector.md)。 | C3.4 |
| <a id="d-30"></a>D-30 | Finger Tree | 実装あり | 計測付き永続2–3 Finger Tree。digit/middle、両端・split/concat、非可換積。strict評価の最悪O(log n)。 [実装](../ordered_structure/finger_tree.hpp) / [説明・検証](ordered_structure/finger_tree.md)。 | C3.5 |
| <a id="d-31"></a>D-31 | Partially Retroactive Priority Queue | 実装あり | 登録時刻のinsert/delete-minをbridge法で編集。更新O(log m)、現在min/sizeはO(1)、無効履歴の編集拒否。 [実装](../ordered_structure/partially_retroactive_priority_queue.hpp) / [説明・検証](ordered_structure/partially_retroactive_priority_queue.md)。 | C3.5 |
| <a id="d-32"></a>D-32 | 完全永続Lazy Balanced BST | 実装あり | 完全永続のlazy動的列。過去版分岐・affine作用・反転・split/concat、サイズ比RBST。 [実装](../ordered_structure/persistent_lazy_rbst.hpp) / [説明・検証](ordered_structure/persistent_lazy_rbst.md)。 | C3.5 |
| <a id="d-33"></a>D-33 | 三次元以上の直交範囲クエリ | 実装あり | 1〜10次元の静的点集合の半開box個数・重み和。D-35を共有。 [実装](../offline_query/orthogonal_range_query.hpp) / [説明・検証](offline_query/orthogonal_range_query.md)。 | C3.6 |
| <a id="d-34"></a>D-34 | 特殊な動的最近傍探索 | 実装あり | 仕様: 1〜10次元の点追加・削除・移動と二乗Euclid最近傍。KD＋更新bufferの再構築、最悪線形検索。 [実装](../range_query/dynamic_nearest_neighbor.hpp) / [説明・検証](range_query/dynamic_nearest_neighbor.md)。 | C3.6 |
| <a id="d-35"></a>D-35 | 汎用多次元Fenwick Tree | 実装あり | 登録座標上の疎な再帰Fenwick。1〜10次元、点加算・prefix和・box和。 [実装](../range_query/fenwick_tree_nd.hpp) / [説明・検証](range_query/fenwick_tree_nd.md)。 | C3.6 |

## グラフ・フロー・被覆（3.7〜3.10）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="g-01"></a>G-01 | 区間辺グラフ / Segment Tree Graph | 実装あり | 区間辺グラフの公開API・復元・条件・検証を追加。[実装](../graph_basic/segment_tree_graph.hpp) / [説明・検証](graph_basic/segment_tree_graph.md)。 | 追加 |
| <a id="g-02"></a>G-02 | 二部最小頂点被覆・最大独立集合の復元 | 実装あり | 二部最小頂点被覆・最大独立集合の公開API・復元・条件・検証を追加。[実装](../graph_flow/bipartite_vertex_cover.hpp) / [説明・検証](graph_flow/bipartite_vertex_cover.md)。 | 追加 |
| <a id="g-03"></a>G-03 | 最小辺被覆の復元 | 実装あり | 最小辺被覆の公開API・復元・条件・検証を追加。[実装](../graph_combinatorics/minimum_edge_cover.hpp) / [説明・検証](graph_combinatorics/minimum_edge_cover.md)。 | 追加 |
| <a id="g-04"></a>G-04 | Gomory–Hu Tree | 実装あり | Gomory–Hu Treeの公開API・復元・条件・検証を追加。[実装](../graph_flow/gomory_hu_tree.hpp) / [説明・検証](graph_flow/gomory_hu_tree.md)。 | 追加 |
| <a id="g-05"></a>G-05 | st-Numbering / Bipolar Orientation | 実装あり | st-Numbering・Bipolar Orientationの公開API・復元・条件・検証を追加。[実装](../graph_connectivity/st_numbering.hpp) / [説明・検証](graph_connectivity/st_numbering.md)。 | 追加 |
| <a id="g-06"></a>G-06 | Minimum Mean Cycle / Karp | 実装あり | 最小平均閉路 / Karpの公開API・復元・条件・検証を追加。[実装](../graph_basic/minimum_mean_cycle.hpp) / [説明・検証](graph_basic/minimum_mean_cycle.md)。 | 追加 |
| <a id="g-07"></a>G-07 | 次数列の判定・グラフ構成 | 実装あり | 次数列の判定と構成の公開API・復元・条件・検証を追加。[実装](../graph_combinatorics/degree_sequence.hpp) / [説明・検証](graph_combinatorics/degree_sequence.md)。 | 追加 |
| <a id="g-08"></a>G-08 | Borůvka・暗黙グラフのMST | 実装あり | Borůvka・暗黙MST・XOR-MSTの公開API・復元・条件・検証を追加。[実装](../graph_basic/boruvka.hpp) / [説明・検証](graph_basic/boruvka.md)。 | 追加 |
| <a id="g-09"></a>G-09 | Incremental Minimum Spanning Forest | 実装あり | Incremental Minimum Spanning Forestの公開API・復元・条件・検証を追加。[実装](../graph_basic/incremental_msf.hpp) / [説明・検証](graph_basic/incremental_msf.md)。 | 追加 |
| <a id="g-10"></a>G-10 | 無向なもりグラフの分解 | 実装あり | 無向なもり分解の公開API・復元・条件・検証を追加。[実装](../graph_connectivity/unicyclic_decomposition.hpp) / [説明・検証](graph_connectivity/unicyclic_decomposition.md)。 | 追加 |
| <a id="g-11"></a>G-11 | 最大反鎖の復元 | 実装あり | DAGの最大反鎖の公開API・復元・条件・検証を追加。[実装](../graph_combinatorics/dag_maximum_antichain.hpp) / [説明・検証](graph_combinatorics/dag_maximum_antichain.md)。 | 追加 |
| <a id="g-12"></a>G-12 | 一般単純グラフのΔ+1辺彩色 / Vizing | 実装あり | 一般単純グラフのΔ+1辺彩色の公開API・復元・条件・検証を追加。[実装](../graph_combinatorics/vizing_edge_coloring.hpp) / [説明・検証](graph_combinatorics/vizing_edge_coloring.md)。 | 追加 |
| <a id="g-13"></a>G-13 | 差分制約の完成API | 実装あり | 差分制約の公開API・復元・条件・検証を追加。[実装](../graph_basic/difference_constraints.hpp) / [説明・検証](graph_basic/difference_constraints.md)。 | 追加 |
| <a id="g-14"></a>G-14 | 負閉路そのものの抽出 | 実装あり | 負閉路の抽出の公開API・復元・条件・検証を追加。[実装](../graph_basic/negative_cycle.hpp) / [説明・検証](graph_basic/negative_cycle.md)。 | 追加 |
| <a id="g-15"></a>G-15 | 最小費用の辺素/頂点素経路 / Suurballe・Bhandari | 実装あり | 最小費用の辺素・頂点素パスの公開API・復元・条件・検証を追加。[実装](../graph_combinatorics/min_cost_disjoint_paths.hpp) / [説明・検証](graph_combinatorics/min_cost_disjoint_paths.md)。 | 追加 |
| <a id="g-16"></a>G-16 | 高速最小費用流 / Network Simplex・Cost Scaling | 未実装 | 既存SSPとは別のアルゴリズム。採用する方式・容量/費用範囲・性能を決める。 確認先: [graph_flow/min_cost_b_flow.hpp](../graph_flow/min_cost_b_flow.hpp)。 | 追加 |
| <a id="g-20"></a>G-20 | 高速な特殊重み最短路 | 要仕様化 | Dial/Radix/0-1 BFSは既存。これら以外の重み条件と目標計算量を先に定める。 確認先: [graph_basic/dial_shortest_path.hpp](../graph_basic/dial_shortest_path.hpp) / [graph_basic/radix_heap_dijkstra.hpp](../graph_basic/radix_heap_dijkstra.hpp)。 | C3.7 |
| <a id="g-21"></a>G-21 | グラフ制約付きShortest Nonzero Path | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.7 |
| <a id="g-22"></a>G-22 | Minimum Diameter Spanning Tree | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.7 |
| <a id="g-23"></a>G-23 | SPQR Tree | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.8 |
| <a id="g-24"></a>G-24 | 一般グラフの木幅分解 | 一部対応 | 幅2の分解だけ既存。任意幅の探索・分解/証明を補う。 確認先: [graph_connectivity/tree_decomposition_width_2.hpp](../graph_connectivity/tree_decomposition_width_2.hpp)。 | C3.8 |
| <a id="g-25"></a>G-25 | 特殊なグラフの標準形 | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.8 |
| <a id="g-26"></a>G-26 | 一般グラフの最大重みマッチング（Blossomの重み付き版） | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.9 |
| <a id="g-27"></a>G-27 | Rank-Maximal Matching | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.9 |
| <a id="g-28"></a>G-28 | 特殊な動的フロー | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.9 |
| <a id="g-29"></a>G-29 | ラベル付き連結グラフ数 | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.10 |
| <a id="g-30"></a>G-30 | 強連結有向グラフ数 | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.10 |
| <a id="g-31"></a>G-31 | Euler閉路数（BEST定理） | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.10 |
| <a id="g-32"></a>G-32 | 一般的なグラフ同型 | 未実装 | 1-WL filterは未確定を返す。一般グラフの厳密判定/対応復元は未対応。 確認先: [graph_combinatorics/graph_isomorphism_filter.hpp](../graph_combinatorics/graph_isomorphism_filter.hpp)。 | C3.10 |
| <a id="g-33"></a>G-33 | 特殊なグラフ分解 | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.10 |

## 木（3.11）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="t-01"></a>T-01 | 木の中心 / Tree Center | 実装あり | 重みなし中心、非負重みの頂点中心全件と辺内部の連続中心。整数の2倍表現で半整数を保持。 [実装](../tree/tree_center.hpp) / [説明・検証](tree/tree_center.md)。 | 追加 |
| <a id="t-02"></a>T-02 | Prüfer Code | 実装あり | 最小葉の符号化/復号。nを明示し1/2を区別、不正な木・長さ・ラベルを拒否。 [実装](../tree/prufer_code.hpp) / [説明・検証](tree/prufer_code.md)。 | 追加 |
| <a id="t-03"></a>T-03 | 根変更時の部分木クエリ補助 | 実装あり | 既存HLDを共有し、指定根の部分木を成分内の高々2区間へO(log n)で分解。 [実装](../tree/rerooted_subtree.hpp) / [説明・検証](tree/rerooted_subtree.md)。 | 追加 |
| <a id="t-20"></a>T-20 | Dynamic Top Tree | 実装あり | 辺のlink/cutを伴うrake/compress、根指定の成分/部分木cluster、非可換compress。所有権とノード再利用。 [実装](../tree/dynamic_top_tree.hpp) / [説明・検証](tree/dynamic_top_tree.md)。 | C3.11 |
| <a id="t-21"></a>T-21 | Lazy Top Tree | 実装あり | 動的topology上のパス/部分木/成分へのlazy作用。spineと枝の条件を明記し、代入・affine合成を検証。 [実装](../tree/lazy_top_tree.hpp) / [説明・検証](tree/lazy_top_tree.md)。 | C3.11 |
| <a id="t-22"></a>T-22 | 動的部分木集約 | 実装あり | 可換ActedMonoidの指定根subtree_prod/apply。link/cut、path/componentも共通Top Treeで処理。 [実装](../tree/dynamic_subtree_aggregate.hpp) / [説明・検証](tree/dynamic_subtree_aggregate.md)。 | C3.11 |
| <a id="t-23"></a>T-23 | Fully Dynamic Tree DP | 実装あり | link/cutと頂点更新を伴う汎用DP。独立集合の重み付き数え上げ・根依存の距離和を公開。 [実装](../tree/fully_dynamic_tree_dp.hpp) / [説明・検証](tree/fully_dynamic_tree_dp.md)。 | C3.11 |

## DP・厳密探索（3.12）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="p-01"></a>P-01 | Profile / Plug / Frontier DP | 未実装 | 境界状態・連結性の正規化・遷移、経路/閉路条件の完成例。 | 追加 |
| <a id="p-02"></a>P-02 | Bit-parallel LCS | 未実装 | 通常LCS、Four RussiansのBoolean積とは別の高速化。 | 追加 |
| <a id="p-03"></a>P-03 | Bit-parallel Edit Distance / Myers | 未実装 | 対応編集操作・単位コスト・複数wordと長さ制限。 | 追加 |
| <a id="p-04"></a>P-04 | Exact Cover / Algorithm X / Dancing Links | 未実装 | 列制約と解集合の列挙/一解復元。 | 追加 |
| <a id="p-05"></a>P-05 | A* | 未実装 | 状態、許容/整合ヒューリスティック、再探索と経路復元。 | 追加 |
| <a id="p-06"></a>P-06 | IDA* | 未実装 | 反復閾値・深さ/費用と循環除去、最適性の条件。 | 追加 |
| <a id="p-07"></a>P-07 | 双方向状態空間探索 | 未実装 | 逆遷移・探索の合流・最短解の復元。 | 追加 |
| <a id="p-08"></a>P-08 | 汎用SAT Solver | 未実装 | DPLL/CDCL等。既存2-SATとは別。 | 追加 |
| <a id="p-09"></a>P-09 | ナップサック・部分和の復元 | 一部対応 | 0-1/完全/個数制限/Bitsetの品物IDや個数の復元。既存KnapsackResultはdpと最適値。 確認先: [dp_optimization/knapsack_common.hpp](../dp_optimization/knapsack_common.hpp) / [dp_optimization/bitset_knapsack.hpp](../dp_optimization/bitset_knapsack.hpp)。 | 追加 |
| <a id="p-10"></a>P-10 | 最適解の個数を返すDP API | 要仕様化 | LIS/LCS/Knapsack等の対象、添字列と値列の同一視、空解・同点の数え方を先に確定する。 確認先: [dp_optimization/lis.hpp](../dp_optimization/lis.hpp) / [dp_optimization/lcs.hpp](../dp_optimization/lcs.hpp)。 | 追加 |
| <a id="p-20"></a>P-20 | 高度なMonge行列積 | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.12 |
| <a id="p-21"></a>P-21 | 複雑なSlope Trick | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.12 |
| <a id="p-22"></a>P-22 | 動的な最適化DP | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.12 |
| <a id="p-23"></a>P-23 | 特殊な高速ナップサック | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.12 |

## 数論（3.13〜3.14）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="n-01"></a>N-01 | 区間篩 / Segmented Sieve | 未実装 | 大きいLに対する短区間の素数列挙・素因数分解。通常の篩とは別。 | 追加 |
| <a id="n-02"></a>N-02 | 乗法位数・Carmichael関数 | 未実装 | 任意の可逆元の位数とλ(m)、非可逆入力の扱い。 | 追加 |
| <a id="n-03"></a>N-03 | Min of Mod of Linear | 未実装 | 区間上の(a*i+b) mod mの最小値と実現添字。 | 追加 |
| <a id="n-04"></a>N-04 | 重み付き/多項式Floor Sum・万能Euclid | 未実装 | 添字/床値のべき付き総和、モノイド一般化。通常floor_sumは既存。 確認先: [number_theory_basic/floor_sum.hpp](../number_theory_basic/floor_sum.hpp)。 | 追加 |
| <a id="n-05"></a>N-05 | Batch Inversion | 未実装 | 積と一回の逆元から全逆元を求める。非unit・0・合成数法の条件。 | 追加 |
| <a id="n-06"></a>N-06 | 素数冪/CRTによる大引数の任意mod二項係数 | 一部対応 | 合成数法のPascal表と、固定Nの全kを出す旧実装は既存。巨大n・素数冪・多数query版を補う。 確認先: [combinatorics/binomial.hpp](../combinatorics/binomial.hpp) / [Combination/Combination_ModNotPrime.hpp](../Combination/Combination_ModNotPrime.hpp) / [number_theory_basic/lucas.hpp](../number_theory_basic/lucas.hpp)。 | 追加 |
| <a id="n-07"></a>N-07 | 巨大なnの階乗 mod p | 未実装 | 線形表の上限を超える引数を、標本点シフト等で計算するAPI。 確認先: [combinatorics/factorials.hpp](../combinatorics/factorials.hpp)。 | 追加 |
| <a id="n-08"></a>N-08 | 法固定・前計算再利用の離散対数 | 未実装 | 単発の非互いに素も扱うBSGSは既存。固定modの複数問い合わせで前計算を共有。 確認先: [number_theory_basic/discrete_log.hpp](../number_theory_basic/discrete_log.hpp)。 | 追加 |
| <a id="n-09"></a>N-09 | 商列挙prefix表上のDirichlet積 | 未実装 | 配列版dirichlet_convolutionとは別の表現・計算量。 確認先: [number_theory_convolution/dirichlet_convolution.hpp](../number_theory_convolution/dirichlet_convolution.hpp)。 | 追加 |
| <a id="n-10"></a>N-10 | Dirichlet逆元 | 未実装 | 配列版と商列挙prefix表版の双方、f(1)が可逆である条件。 | 追加, C3.14 |
| <a id="n-20"></a>N-20 | mod k乗根 | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.13 |
| <a id="n-21"></a>N-21 | Gaussian Integer | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.13 |
| <a id="n-22"></a>N-22 | 連分数・有理数近似 | 一部対応 | Stern–Brocot経路・分数境界は既存。連分数展開・convergentsなど残るAPIを整理。 確認先: [basic_utility/stern_brocot_tree.hpp](../basic_utility/stern_brocot_tree.hpp) / [basic_utility/fraction_binary_search.hpp](../basic_utility/fraction_binary_search.hpp)。 | C3.13 |
| <a id="n-23"></a>N-23 | Pell方程式 | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.13 |
| <a id="n-24"></a>N-24 | 二平方数和の表示 | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.13 |
| <a id="n-25"></a>N-25 | 高速素数和 | 未実装 | prime_countは個数のみ。大きなNまでの素数の和と計算量を別途提供。 確認先: [number_theory_convolution/prime_counting.hpp](../number_theory_convolution/prime_counting.hpp)。 | C3.14 |
| <a id="n-26"></a>N-26 | 一般化されたMin_25 Sieve | 未実装 | Lucy個数・線形篩prefixのmultiplicative_function_sumとは異なる汎用化。 確認先: [number_theory_convolution/multiplicative_function_sum.hpp](../number_theory_convolution/multiplicative_function_sum.hpp)。 | C3.14 |
| <a id="n-27"></a>N-27 | 任意乗法的関数の大規模総和 | 一部対応 | 巨大Nに対する乗法的関数の総和。既存は上限Nまで線形前計算する版。 確認先: [number_theory_convolution/multiplicative_function_sum.hpp](../number_theory_convolution/multiplicative_function_sum.hpp)。 | C3.14 |
| <a id="n-28"></a>N-28 | 特殊なmod和 | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.14 |

## 組合せ・数え上げ（3.15）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="c-01"></a>C-01 | q-二項係数・q-階乗 | 未実装 | qの位数、分母消失、素数法等の条件を定める。 | 追加 |
| <a id="c-02"></a>C-02 | 上下境界付き単調列/格子路の数え上げ | 未実装 | 分割統治＋畳み込みの完成API。 | 追加 |
| <a id="c-03"></a>C-03 | 小さい素数法・巨大nのStirling数 | 未実装 | 第1/第2種。通常の二乗表やFPS一行計算とは別。 | 追加 |
| <a id="c-04"></a>C-04 | ラベルなし木の数え上げ | 未実装 | 根付き/無根、同型類。木同型判定とは別。 | 追加 |
| <a id="c-20"></a>C-20 | RSK対応 | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.15 |
| <a id="c-21"></a>C-21 | Young Tableaux | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.15 |
| <a id="c-22"></a>C-22 | Hook-Length Formula | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.15 |
| <a id="c-23"></a>C-23 | 高度な組合せ恒等式用の変換 | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.15 |

## 畳み込み・多項式・母関数（3.16〜3.19）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="q-01"></a>Q-01 | 多項式拡張GCD・mod逆元 | 未実装 | Bézout係数と剰余環での逆元。通常polynomial_gcdだけでは不足。 確認先: [fps/polynomial_gcd.hpp](../fps/polynomial_gcd.hpp)。 | 追加 |
| <a id="q-02"></a>Q-02 | Polynomial Resultant / Discriminant | 未実装 | 共通根・重根判定、零多項式/定数と標数の規約。 | 追加 |
| <a id="q-03"></a>Q-03 | 有限体上の多項式因数分解 | 未実装 | square-free・次数別・既約因子への分解。単なる根列挙とは別。 | 追加 |
| <a id="q-04"></a>Q-04 | mod 2^64の畳み込み | 未実装 | uint64のwraparound結果。既存のsigned64への厳密復元とは別。 確認先: [convolution/convolution_ll.hpp](../convolution/convolution_ll.hpp)。 | 追加 |
| <a id="q-05"></a>Q-05 | GF(2^64)上の畳み込み | 未実装 | Nimber等の固定体演算を使う畳み込み。整数mod 2^64とは別。 確認先: [game_theory/nimber.hpp](../game_theory/nimber.hpp)。 | 追加 |
| <a id="q-06"></a>Q-06 | 転置原理 / Tellegenの公開部品 | 未実装 | 対象の線形変換・転置作用と前計算の共有を具体化する。 | 追加 |
| <a id="q-07"></a>Q-07 | NTT Doubling | 未実装 | 既存変換値から長さ2倍の変換を構成するAPI。 | 追加 |
| <a id="q-08"></a>Q-08 | Sharp P Subset Sum | 未実装 | 積∏(1+x^s)をlog/expで一括計算する完成用途。 | 追加 |
| <a id="q-09"></a>Q-09 | 多項式の不定和分 / Prefix Sum of Polynomial | 未実装 | 値のprefix和を表す多項式の係数を返す。 | 追加 |
| <a id="q-20"></a>Q-20 | Rader FFT | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.16 |
| <a id="q-21"></a>Q-21 | Bluestein FFT | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.16 |
| <a id="q-22"></a>Q-22 | 高性能64bit NTT | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.16 |
| <a id="q-23"></a>Q-23 | AVX2 NTT | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.16 |
| <a id="q-25"></a>Q-25 | 多変数巡回畳み込み | 一部対応 | 通常のtensor畳み込みは既存。軸ごとの巡回還元を行う公開APIを補う。 確認先: [convolution/multidimensional_convolution.hpp](../convolution/multidimensional_convolution.hpp)。 | C3.16 |
| <a id="q-26"></a>Q-26 | Multiplicative Convolution | 未実装 | 剰余乗法による添字の積で集約する畳み込み。添字i*jが通常の整数であるDirichlet積やGCD/LCM積とは別。 確認先: [number_theory_convolution/dirichlet_convolution.hpp](../number_theory_convolution/dirichlet_convolution.hpp)。 | C3.16 |
| <a id="q-27"></a>Q-27 | Power Projection | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.17 |
| <a id="q-28"></a>Q-28 | 多項式の根の高速計算 | 未実装 | 有限体の根列挙/高速化が対象。数値実根はY-04、全因数分解はQ-03。 | C3.17 |
| <a id="q-29"></a>Q-29 | 高速FPS合成 | 一部対応 | 有限多項式のbaby-step/giant-step版は既存。truncated compositionのより高速な計算量を対象にする。 確認先: [fps/composition.hpp](../fps/composition.hpp)。 | C3.17 |
| <a id="q-30"></a>Q-30 | 特殊なFPS変換 | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.17 |
| <a id="q-31"></a>Q-31 | 多変数FPS（二変数を含む） | 未実装 | 二変数/多変数の切り詰め領域・係数配置・inv等の演算を共通の作業単位へ統合。通常多変数多項式積だけではFPS演算の代用にならない。 | C3.17 |
| <a id="q-33"></a>Q-33 | FPSの微分方程式 | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.17 |
| <a id="q-24"></a>Q-24 | 多項式行列の行列式・Prefix Product | 未実装 | 3.17/3.20の重複を統合。スカラー行列や行列係数の一点評価だけとは区別。 | C3.17, C3.20 |
| <a id="q-34"></a>Q-34 | Composed Sum/Product | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.17 |
| <a id="q-35"></a>Q-35 | 高速中間積 | 一部対応 | 既存middle_productは全積から切出す。中間積専用/転置の高速化を追加。 確認先: [convolution/middle_product.hpp](../convolution/middle_product.hpp)。 | C3.17 |
| <a id="q-36"></a>Q-36 | Holonomic Sequence | 一部対応 | 既知P-recursiveのprefixは実装済み。推定・閉包操作・高速大引数項など高度な対象を確定する。 確認先: [linear_recurrence/p_recursive.hpp](../linear_recurrence/p_recursive.hpp)。 | C3.18 |
| <a id="q-37"></a>Q-37 | Reeds–Sloane | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.18 |
| <a id="q-38"></a>Q-38 | 特殊な母関数の合成 | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.18 |
| <a id="q-39"></a>Q-39 | 線形漸化式の大量項計算 | 一部対応 | prefix逐次生成・単一n項は既存。巨大開始位置からの連続項など高速な大量出力を対象にする。 確認先: [linear_recurrence/coefficient_extraction.hpp](../linear_recurrence/coefficient_extraction.hpp) / [linear_recurrence/infer_nth.hpp](../linear_recurrence/infer_nth.hpp)。 | C3.18 |
| <a id="q-40"></a>Q-40 | SPS Power Projection | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.19 |
| <a id="q-41"></a>Q-41 | Online Set Function Convolution | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.19 |
| <a id="q-42"></a>Q-42 | 特殊な集合関数の変換 | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.19 |

## 線形代数・専門的代数（3.20）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="l-01"></a>L-01 | 任意/合成数modの行列式 | 未実装 | 非unitをpivotにした除算を避ける。既存determinant_modは素数法用。 確認先: [graph_combinatorics/determinant_mod.hpp](../graph_combinatorics/determinant_mod.hpp)。 | 追加 |
| <a id="l-02"></a>L-02 | 整数行列の厳密行列式 | 未実装 | Bareissや多倍長/CRT等。整数に体の除算をそのまま適用しない。 | 追加 |
| <a id="l-03"></a>L-03 | 半環行列・Min-Plus/Max-Plus/Booleanの共通API | 一部対応 | Matrixの通常環演算は既存。加算/積/零/単位を明示して差し替える半環用入口を補う。 確認先: [linear_algebra/matrix_product.hpp](../linear_algebra/matrix_product.hpp) / [linear_algebra/matrix_power.hpp](../linear_algebra/matrix_power.hpp) / [dp_optimization/four_russians.hpp](../dp_optimization/four_russians.hpp)。 | 追加 |
| <a id="l-04"></a>L-04 | Pfaffian | 未実装 | 交代行列・素数標数等の条件と数値範囲。Hafnian/Permanentとは別。 | 追加 |
| <a id="l-05"></a>L-05 | Permanent | 未実装 | Ryser等の指数時間・小規模向けAPI。 | 追加 |
| <a id="l-06"></a>L-06 | Smith Normal Form・整数/一般合同線形系 | 未実装 | 変換行列・不変因子と整数解/合同式の解空間。素数体Gauss–Jordanとは別。 | 追加 |
| <a id="l-07"></a>L-07 | Hermite Normal Form | 未実装 | 整数行列の標準形と変換の規約。 | 追加 |
| <a id="l-08"></a>L-08 | 汎用拡大有限体・GF(2)多項式演算 | 一部対応 | 固定GF(2^64)のNimber演算は既存。任意の既約多項式の表現・乗除/GCD・拡大体を補う。 確認先: [game_theory/nimber.hpp](../game_theory/nimber.hpp) / [linear_algebra/gf2_gauss.hpp](../linear_algebra/gf2_gauss.hpp)。 | 追加 |
| <a id="l-20"></a>L-20 | Frobenius Normal Form | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.20 |
| <a id="l-21"></a>L-21 | 行列のHafnian | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.20 |
| <a id="l-22"></a>L-22 | 高速な特性多項式計算 | 一部対応 | HessenbergのO(n^3)版は既存。追加の速度目標・アルゴリズムを明示する。 確認先: [linear_algebra/characteristic_polynomial.hpp](../linear_algebra/characteristic_polynomial.hpp)。 | C3.20 |

## 文字列・圧縮索引（3.21）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="s-01"></a>S-01 | 部分列オートマトン・異なる部分列数 | 未実装 | next occurrence tableと部分列の照合/数え上げ。部分文字列のSAMとは別。 | 追加 |
| <a id="s-02"></a>S-02 | Lempel–Ziv Factorization | 未実装 | Lyndon/Runsとは別の分解と復元情報。 | 追加 |
| <a id="s-03"></a>S-03 | BWT・逆BWT | 未実装 | 終端/primary index、全byte、可逆性。 | 追加 |
| <a id="s-04"></a>S-04 | FM-index | 未実装 | BWTとrank/selectに基づく圧縮全文索引。 | 追加 |
| <a id="s-05"></a>S-05 | Prefix-Substring LCS / Semi-local LCS | 未実装 | static range LISと共通化できるunit-Monge/seaweed基盤。通常LCSやLongest Common Substringとは別。 | 追加 |
| <a id="s-06"></a>S-06 | Border Tree・周期の全列挙 | 一部対応 | prefix function/Runsは既存。失敗木と全periodを返す完成APIを補う。 確認先: [string_algorithm/prefix_function.hpp](../string_algorithm/prefix_function.hpp) / [string_algorithm/runs_enumeration.hpp](../string_algorithm/runs_enumeration.hpp)。 | 追加 |
| <a id="s-07"></a>S-07 | SAM utilitiesの未対応分 | 一部対応 | k番目の異なる部分文字列、SAMでの全出現位置列挙。contains/count/first/distinct/LCSは既存。 確認先: [string_algorithm/suffix_automaton.hpp](../string_algorithm/suffix_automaton.hpp)。 | 追加 |
| <a id="s-20"></a>S-20 | Double-Ended Palindromic Tree | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.21 |
| <a id="s-21"></a>S-21 | Online Z Algorithm | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.21 |
| <a id="s-22"></a>S-22 | Generalized Suffix Automaton | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.21 |
| <a id="s-23"></a>S-23 | 高度な文字列周期問題 | 要仕様化 | Runs・最小回転等は既存。追加queryやオンライン/圧縮条件を決める。 確認先: [string_algorithm/runs_enumeration.hpp](../string_algorithm/runs_enumeration.hpp)。 | C3.21 |

## 計算幾何・平面グラフ（3.6・3.22）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="h-01"></a>H-01 | 長方形の和集合面積 | 未実装 | 軸平行矩形のsweep＋被覆長。rectangle sum queryとは別。 | 追加 |
| <a id="h-02"></a>H-02 | 長方形の和集合周長 | 未実装 | 被覆の連結区間数と同時刻イベントを扱う。 | 追加 |
| <a id="h-03"></a>H-03 | 円の接線の新API整備 | 整備待ち | 旧tangent_CP/tangent_CCあり。点からの接線と2円の共通接線を新数値規約へ移し、退化と出力を検証。 確認先: [geometry/2D_geometry_template.hpp](../geometry/2D_geometry_template.hpp)。 | 追加 |
| <a id="h-04"></a>H-04 | 三角形の外接円・内接円等の新API整備 | 整備待ち | 旧incircle/circumcircle/orthocenterあり。MEC内部through_threeだけを公開APIとは数えない。 確認先: [geometry/2D_geometry_template.hpp](../geometry/2D_geometry_template.hpp) / [computational_geometry/minimum_enclosing_circle.hpp](../computational_geometry/minimum_enclosing_circle.hpp)。 | 追加 |
| <a id="h-05"></a>H-05 | 多角形の重心の新API整備 | 整備待ち | 旧Centroidあり。面積0、向き、整数/有理/浮動の条件を整理。 確認先: [geometry/2D_geometry_template.hpp](../geometry/2D_geometry_template.hpp)。 | 追加 |
| <a id="h-06"></a>H-06 | 円と多角形の共通部分面積 | 一部対応 | 旧intersection_area_CPには改善注記あり。退化・接触を含む新APIと独立比較を補う。 確認先: [geometry/2D_geometry_template.hpp](../geometry/2D_geometry_template.hpp)。 | 追加 |
| <a id="h-07"></a>H-07 | 2円の共通部分面積の新API整備 | 整備待ち | 旧intersection_area_CCあり。包含・接触・点円の規約と数値検証。 確認先: [geometry/2D_geometry_template.hpp](../geometry/2D_geometry_template.hpp)。 | 追加 |
| <a id="h-08"></a>H-08 | 円の和集合面積 | 未実装 | 交差・包含・重複円を扱う。2円の交点だけとは別。 | 追加 |
| <a id="h-09"></a>H-09 | 平面埋め込みの面列挙・双対グラフ | 未実装 | 偏角順とhalf-edge走査、外側面・非連結/橋の扱い。交点分割はH-12へ。 | 追加 |
| <a id="h-10"></a>H-10 | 非整数領域の格子点計数・整数凸包 | 未実装 | 有理頂点の領域を対象にする。整数多角形用Pickとは別。 | 追加 |
| <a id="h-11"></a>H-11 | 原点を共有する矩形の増分和集合面積 | 未実装 | 特殊形状の追加。一般矩形の任意追加削除と取り違えない。 | 追加 |
| <a id="h-12"></a>H-12 | 線分sweep・全交点列挙・Arrangement | 未実装 | 交差存在判定から交点分割・平面グラフ化までの共通基盤。共線重複・多重点・出力規模を明示。 | 追加 |
| <a id="h-13"></a>H-13 | 凸多角形intersectionの完成API | 一部対応 | 旧intersection_PoPo/面積と新convex cutは既存。新型の交差多角形・点/線分退化・性能保証を補う。 確認先: [geometry/2D_geometry_template.hpp](../geometry/2D_geometry_template.hpp) / [computational_geometry/convex_cut.hpp](../computational_geometry/convex_cut.hpp)。 | 追加 |
| <a id="h-14"></a>H-14 | adaptive/exactな幾何predicates | 一部対応 | 整数orientationは検査付き128bitで既存。全座標域への拡張、浮動入力の適応精度orientation/incircleは未対応。 確認先: [computational_geometry/exact/point.hpp](../computational_geometry/exact/point.hpp) / [computational_geometry/floating/point.hpp](../computational_geometry/floating/point.hpp)。 | 追加 |
| <a id="h-15"></a>H-15 | 三角形内の点の個数クエリ | 未実装 | 多数queryの境界・退化を含む計数。 | 追加 |
| <a id="h-16"></a>H-16 | 凸多角形の全頂点の最遠点 | 未実装 | 各頂点のfurthest neighbor。集合全体のFarthest Pairとは別。 | 追加 |
| <a id="h-17"></a>H-17 | 一般/凸多角形の和集合面積 | 未実装 | 元CのConvex Polygon Union Areaも包含。重複・穴・辺の接触の扱い。 | 追加, C3.22 |
| <a id="h-20"></a>H-20 | Delaunay Triangulation | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.22 |
| <a id="h-21"></a>H-21 | Voronoi Diagram | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.22 |
| <a id="h-22"></a>H-22 | Dynamic Convex Hull | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.22 |
| <a id="h-23"></a>H-23 | Polygon Triangulation | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.22 |
| <a id="h-24"></a>H-24 | 三次元幾何 | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.22 |

## 数値解析（新カテゴリ候補）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="y-01"></a>Y-01 | 実数方程式の求根 | 未実装 | 安定な二次方程式、二分法・Newton・Brent、収束/停止/失敗の条件。 | 追加 |
| <a id="y-02"></a>Y-02 | 数値積分 | 未実装 | Simpson・適応Simpson、誤差/最大再帰/特異点の条件。 | 追加 |
| <a id="y-03"></a>Y-03 | 数値安定化ユーティリティ | 未実装 | Kahan/補償和、Log-Sum-Exp、絶対＋相対誤差比較。幾何ToleranceやGaussのepsilonだけとは区別。 | 追加 |
| <a id="y-04"></a>Y-04 | 多項式の浮動小数点実根列挙 | 未実装 | 有限体の多項式因数分解・高速根計算と分ける。 | 追加 |
| <a id="y-05"></a>Y-05 | 三重対角行列ソルバ | 未実装 | pivot/特異系・数値安定性の規約。 | 追加 |

## LP・マトロイド等（3.23）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="o-01"></a>O-01 | Simplex / 線形計画 | 未実装 | 最適・実行不能・非有界、解ベクトルと数値条件。 | 追加 |
| <a id="o-02"></a>O-02 | 低次元線形計画 | 未実装 | 2D/3Dなど固定次元、非有界・退化の判定。 | 追加 |
| <a id="o-03"></a>O-03 | Matroid Union / Matroid Partition | 未実装 | 独立集合への分配・分割と復元。PartitionMatroidとは別。 | 追加 |
| <a id="o-04"></a>O-04 | 分数計画 | 未実装 | 比率の二分探索/Dinkelbach、分母の符号と停止条件。 | 追加 |
| <a id="o-20"></a>O-20 | Weighted Matroid Intersection | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.23 |
| <a id="o-21"></a>O-21 | Base Polyhedron | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.23 |
| <a id="o-22"></a>O-22 | Submodular Optimization | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.23 |
| <a id="o-23"></a>O-23 | 特殊な整数計画法 | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.23 |

## 高度なゲーム理論（3.24）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="b-20"></a>B-20 | 超限的なゲーム値・高度な組合せゲーム理論 | 要仕様化 | 既存Partizanは有限short、Nimberはuint64。超限的値など対象の表現と演算を具体化する。 確認先: [game_theory/partizan_game.hpp](../game_theory/partizan_game.hpp) / [game_theory/nimber.hpp](../game_theory/nimber.hpp)。 | C3.24 |

## 探索・ヒューリスティック（3.25）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="e-01"></a>E-01 | 汎用Undo Manager・score検算 | 一部対応 | 履歴・snapshot・複合move・例外時復元・全再計算との照合。delta/apply/undo付きSAは既存。 確認先: [heuristic_search/incremental_score.hpp](../heuristic_search/incremental_score.hpp)。 | 追加 |
| <a id="e-02"></a>E-02 | IndexSet | 未実装 | O(1) insert/erase/一様ランダム選択、添字の寿命。 | 追加 |
| <a id="e-03"></a>E-03 | TSP近傍 | 未実装 | 2-opt / Or-opt、delta・apply・undoの完成用途。 | 追加 |
| <a id="e-04"></a>E-04 | SA受理判定の高速化 | 未実装 | 対数table等、確率誤差・乱数消費・性能の実測。通常exp採否は既存。 | 追加 |
| <a id="e-05"></a>E-05 | 差分更新型Beam Search | 未実装 | 探索木DFSでapply/undo、最良状態の復元。重複除去Beamとは別。 | 追加 |
| <a id="e-06"></a>E-06 | Chokudai Search | 未実装 | 深さ別queueと時間予算・解の保持。 | 追加 |
| <a id="e-20"></a>E-20 | 多点並列探索 | 未実装 | 同機能の公開実装は確認できない。元Cの独立した追加対象。 | C3.25 |
| <a id="e-21"></a>E-21 | 特殊な確率分布の高速サンプリング | 要仕様化 | 具体的な操作・入力条件・性能目標を確定する。類似する既存A/Bの全体を未実装扱いにはしない。 | C3.25 |

## 構文解析・オートマトン（新カテゴリ候補）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="x-01"></a>X-01 | 再帰下降parser・構文木のライブラリ化 | 整備待ち | 旧parsingは問題別コードとmain/重複定義を含む。単独API、単項演算・優先度・失敗位置と型を整備。 確認先: [parsing/parsing_template.hpp](../parsing/parsing_template.hpp) / [parsing/parse_tree.hpp](../parsing/parse_tree.hpp)。 | 追加 |
| <a id="x-02"></a>X-02 | DFA/NFAの共通操作 | 未実装 | DFA最小化・積、NFA→DFA、状態爆発と受理条件。既存digit_dpへの接続。 | 追加 |
| <a id="x-03"></a>X-03 | 正規表現→NFA / Thompson構成 | 未実装 | 対応演算・空語・エスケープ等を明示。 | 追加 |

## 実装済みのため残作業から除外した候補

| ID | 候補 | 実装と判定 |
|---|---|---|
| <a id="z-01"></a>Z-01 | Segtree / Lazy Segtreeのmax_right・min_left | [range_query/segtree.hpp](../range_query/segtree.hpp) / [range_query/lazy_segtree.hpp](../range_query/lazy_segtree.hpp) / [verify/range_query/unit_test.cpp](../verify/range_query/unit_test.cpp)。両方向・非可換順序・空境界の検証あり。 |
| <a id="z-02"></a>Z-02 | RollbackDSU＋時間軸セグ木 / OfflineDynamicConnectivity | [union_find/rollback_disjoint_set_union.hpp](../union_find/rollback_disjoint_set_union.hpp) / [offline_query/offline_dynamic_connectivity.hpp](../offline_query/offline_dynamic_connectivity.hpp)。add/remove/query、無向多重辺・再solve・rollback。汎用時間軸APIの不足だけD-09へ。 |
| <a id="z-03"></a>Z-03 | Merge Sort Tree | [range_query/segtree_fractional_cascading.hpp](../range_query/segtree_fractional_cascading.hpp)。各nodeの整列列と子prefixを保持するFC拡張版でrange_freqを公開済み。別名ファイルがなくても未実装ではない。 |
| <a id="z-04"></a>Z-04 | Dynamic Sequence Treap | [ordered_structure/implicit_treap.hpp](../ordered_structure/implicit_treap.hpp) / [ordered_structure/lazy_reversible_treap.hpp](../ordered_structure/lazy_reversible_treap.hpp)。挿入/削除・反転・区間集約/作用あり。Splayという別方式の追加は必要機能の未実装と数えない。 |
| <a id="z-05"></a>Z-05 | SCC condensation DAG | [graph_connectivity/condensation.hpp](../graph_connectivity/condensation.hpp)。頂点成分・縮約DAG・元辺対応、重複除去を公開済み。 |
| <a id="z-06"></a>Z-06 | feasible circulation / min-cost circulation | [graph_flow/lower_bound_flow.hpp](../graph_flow/lower_bound_flow.hpp) / [graph_flow/min_cost_circulation.hpp](../graph_flow/min_cost_circulation.hpp) / [graph_flow/min_cost_b_flow.hpp](../graph_flow/min_cost_b_flow.hpp)。上下限の実行可能循環と負費用を含む最小費用循環あり。高速別方式のみG-16。 |
| <a id="z-07"></a>Z-07 | 非可換HLD | [tree/heavy_light_decomposition.hpp](../tree/heavy_light_decomposition.hpp) / [verify/graph_tree_test.cpp](../verify/graph_tree_test.cpp)。u→vの区間順とreversedを保持。順/逆積を使う構造と合成可能。集約器を内蔵していないことは未対応を意味しない。 |
| <a id="z-08"></a>Z-08 | 木の重心・k番目の祖先/パス頂点 | [tree/centroid_decomposition.hpp](../tree/centroid_decomposition.hpp) / [tree/doubling_lca.hpp](../tree/doubling_lca.hpp) / [tree/binary_lifting.hpp](../tree/binary_lifting.hpp)。tree_centroids、kth_ancestor、jump(u,v,k)。中心とPrüferだけT-01/T-02へ。 |
| <a id="z-09"></a>Z-09 | Digit DP framework | [dp_optimization/digit_dp.hpp](../dp_optimization/digit_dp.hpp)。state/transition/accept、tight/startedを持つ汎用十進桁DP。 |
| <a id="z-10"></a>Z-10 | modint連携の行列 | [linear_algebra/matrix.hpp](../linear_algebra/matrix.hpp) / [linear_algebra/matrix_product.hpp](../linear_algebra/matrix_product.hpp) / [linear_algebra/gauss_jordan.hpp](../linear_algebra/gauss_jordan.hpp) / [verify/linear_algebra/modint.hpp](../verify/linear_algebra/modint.hpp)。既存Matrix<T>は独自mod型で検証済み。ライブラリ共通modintの不足はF-01/F-02に一度だけ計上。 |
| <a id="z-11"></a>Z-11 | RationalGeneratingFunction | [linear_recurrence/rational_generating_function.hpp](../linear_recurrence/rational_generating_function.hpp)。約分・分母正規化、和/積、prefix/第n係数を公開済み。 |
| <a id="z-12"></a>Z-12 | LCE / substring compare API | [string_algorithm/multi_string_compare.hpp](../string_algorithm/multi_string_compare.hpp) / [string_algorithm/rolling_hash.hpp](../string_algorithm/rolling_hash.hpp) / [string_algorithm/suffix_tree.hpp](../string_algorithm/suffix_tree.hpp)。厳密lcp/compare、ハッシュLCP、occurrencesを公開済み。SAMの追加utilitiesのみS-07。 |
| <a id="z-13"></a>Z-13 | 正規化された厳密有理数 | [computational_geometry/detail/wide.hpp](../computational_geometry/detail/wide.hpp)。geometry::exact::Rationalに符号統一・約分・検査付き四則演算・積を使わない比較。汎用入口への整理と多倍長化は別。 |
| <a id="z-14"></a>Z-14 | 整数の厳密orientation・線分判定 | [computational_geometry/exact/point.hpp](../computational_geometry/exact/point.hpp) / [computational_geometry/exact/primitives.hpp](../computational_geometry/exact/primitives.hpp)。検査付き128bitで公開。任意範囲や浮動適応精度はH-14へ。 |
| <a id="z-15"></a>Z-15 | 線分追加Li Chao | [dp_optimization/li_chao_tree.hpp](../dp_optimization/li_chao_tree.hpp)。既知座標、add_line/add_segment、min/maxを公開済み。 |
| <a id="z-16"></a>Z-16 | matching/flowの基本復元 | [graph_flow/bipartite_matching.hpp](../graph_flow/bipartite_matching.hpp) / [graph_flow/dinic.hpp](../graph_flow/dinic.hpp) / [combinatorial_optimization/min_cost_flow.hpp](../combinatorial_optimization/min_cost_flow.hpp)。pairs/左右mate、元辺ごとのflow、min-cut、費用流量slopeあり。被覆・負閉路・費用付き素経路の不足だけGへ。 |
| <a id="z-17"></a>Z-17 | LIS・LCSの復元 | [dp_optimization/lis.hpp](../dp_optimization/lis.hpp) / [dp_optimization/lcs.hpp](../dp_optimization/lcs.hpp)。indices / matchesを返す。Knapsack側の不足のみP-09へ。 |
| <a id="z-18"></a>Z-18 | 厳密signed64整数畳み込み | [convolution/convolution_ll.hpp](../convolution/convolution_ll.hpp) / [verify/convolution/convolution_ll.test.cpp](../verify/convolution/convolution_ll.test.cpp)。範囲検査付きCRT復元あり。任意精度・uint64 wraparound・GF畳み込みとは区別。 |
| <a id="z-19"></a>Z-19 | delta/apply/undo付きscore framework | [heuristic_search/incremental_score.hpp](../heuristic_search/incremental_score.hpp)。棄却/accept例外のundoと差分SAあり。履歴managerと検算の共通化だけE-01。 |
| <a id="z-20"></a>Z-20 | 閉路を含む有限状態ゲーム | [game_theory/graph_game.hpp](../game_theory/graph_game.hpp)。win/lose/draw、手・終局距離を公開済み。 |
| <a id="z-21"></a>Z-21 | 連立一次方程式の特解＋零空間・実数pivot消去 | [linear_algebra/gauss_jordan.hpp](../linear_algebra/gauss_jordan.hpp)。有限体/有理型/浮動のAPIと解空間を公開済み。合成数や整数の一般系はL-06。 |
| <a id="z-22"></a>Z-22 | 非互いに素の場合を含む離散対数 | [number_theory_basic/discrete_log.hpp](../number_theory_basic/discrete_log.hpp)。gcd剥がし＋BSGS。法固定で前計算を再利用する追加はN-08。 |
| <a id="z-23"></a>Z-23 | 静的MEX | [game_theory/grundy.hpp](../game_theory/grundy.hpp)。grundy_mex(vector<int>)で非負の最小欠損値を求め、負値は無視。一般Mexという名前の追加を別実装と数えない。 |
| <a id="z-24"></a>Z-24 | 個別差分/境界テスト・Sanitizer・単独include検証・API規約 | [verify/run_b_21_25.py](../verify/run_b_21_25.py) / [verify/b_21_25/string_test.cpp](../verify/b_21_25/string_test.cpp) / [HANDOFF.md](../HANDOFF.md)。既に多数あり。CI・共通stress runner・生成器・可視化の不足のみVへ。 |
| <a id="z-25"></a>Z-25 | 通常のP-recursive数列（元3.15 C） | [linear_recurrence/p_recursive.hpp](../linear_recurrence/p_recursive.hpp)。既知の多項式係数漸化式からのprefix生成は3.18 Bで実装済み。高度なHolonomic操作は別の元C項目として残す。 |
| <a id="z-26"></a>Z-26 | Fenwick lower_boundの単調性・FPSのunit/標数条件等の基本契約 | [docs/range_query/fenwick_tree.md](../docs/range_query/fenwick_tree.md) / [docs/fps/README.md](../docs/fps/README.md) / [docs/basic_utility/nonmonotone_search.md](../docs/basic_utility/nonmonotone_search.md) / [docs/dp_optimization/min_plus_convolution_b.md](../docs/dp_optimization/min_plus_convolution_b.md) / [docs/game_theory/periodic_grundy.md](../docs/game_theory/periodic_grundy.md)。前提・条件付き高速化・周期候補/証明済み周期の区別は記載済み。 |
| <a id="z-27"></a>Z-27 | 区間Union-Findの区別・成分要素列挙 | [union_find/interval_disjoint_set_union.hpp](../union_find/interval_disjoint_set_union.hpp) / [union_find/parallel_disjoint_set_union.hpp](../union_find/parallel_disjoint_set_union.hpp) / [union_find/disjoint_set_union.hpp](../union_find/disjoint_set_union.hpp)。区間内の併合と対応位置の一括併合を別API化。処理済み境界のskipは区間併合の内部で実装、groups()も公開。汎用callback/付随情報復元のみD-10。 |

## 元Cの全件対応表

元のCは101掲載箇所。ここは残作業の二重掲載ではなく、元の各文言から上の一意なIDへの参照です。多項式行列の行列式は3.17/3.20を同じQ-24に統合しています。

| 元カテゴリ | 元の表記 | 統合先 |
|---|---|---|
| 3.1 C | 部分永続の探索補助 | [U-20](#u-20) |
| 3.1 C | Retroactiveな操作管理 | [U-21](#u-21) |
| 3.1 C | 特殊な整数列挙 | [U-22](#u-22) |
| 3.2 C | 非可換群上のPotentialized DSU | [D-20](#d-20) |
| 3.2 C | 一般的なFully Dynamic Connectivity | [D-21](#d-21) |
| 3.3 C | Kinetic Segment Tree | [D-22](#d-22) |
| 3.3 C | RBST-based Segment Tree | [D-23](#d-23) |
| 3.3 C | 複雑な範囲変換をサポートするBeats | [D-24](#d-24) |
| 3.4 C | Static Range Mode | [D-25](#d-25) |
| 3.4 C | Static Range LIS | [D-26](#d-26) |
| 3.4 C | 静的転倒数クエリ | [D-27](#d-27) |
| 3.4 C | 動的Wavelet Matrix | [D-28](#d-28) |
| 3.4 C | 特殊なRank/Select構造 | [D-29](#d-29) |
| 3.5 C | Finger Tree | [D-30](#d-30) |
| 3.5 C | Partially Retroactive Priority Queue | [D-31](#d-31) |
| 3.5 C | 完全永続Lazy Balanced BST | [D-32](#d-32) |
| 3.6 C | 三次元以上の直交範囲クエリ | [D-33](#d-33) |
| 3.6 C | 特殊な動的最近傍探索 | [D-34](#d-34) |
| 3.6 C | 汎用多次元Fenwick Tree | [D-35](#d-35) |
| 3.7 C | 高速な特殊重み最短路 | [G-20](#g-20) |
| 3.7 C | グラフ制約付きShortest Nonzero Path | [G-21](#g-21) |
| 3.7 C | Minimum Diameter Spanning Tree | [G-22](#g-22) |
| 3.8 C | SPQR Tree | [G-23](#g-23) |
| 3.8 C | 一般グラフの木幅分解 | [G-24](#g-24) |
| 3.8 C | 特殊なグラフの標準形 | [G-25](#g-25) |
| 3.9 C | 一般グラフの最大重みマッチング（Blossomの重み付き版） | [G-26](#g-26) |
| 3.9 C | Rank-Maximal Matching | [G-27](#g-27) |
| 3.9 C | 特殊な動的フロー | [G-28](#g-28) |
| 3.10 C | ラベル付き連結グラフ数 | [G-29](#g-29) |
| 3.10 C | 強連結有向グラフ数 | [G-30](#g-30) |
| 3.10 C | Euler閉路数（BEST定理） | [G-31](#g-31) |
| 3.10 C | 一般的なグラフ同型 | [G-32](#g-32) |
| 3.10 C | 特殊なグラフ分解 | [G-33](#g-33) |
| 3.11 C | Dynamic Top Tree | [T-20](#t-20) |
| 3.11 C | Lazy Top Tree | [T-21](#t-21) |
| 3.11 C | 動的部分木集約 | [T-22](#t-22) |
| 3.11 C | Fully Dynamic Tree DP | [T-23](#t-23) |
| 3.12 C | 高度なMonge行列積 | [P-20](#p-20) |
| 3.12 C | 複雑なSlope Trick | [P-21](#p-21) |
| 3.12 C | 動的な最適化DP | [P-22](#p-22) |
| 3.12 C | 特殊な高速ナップサック | [P-23](#p-23) |
| 3.13 C | mod k乗根 | [N-20](#n-20) |
| 3.13 C | 64bit高性能modint | [F-04](#f-04) |
| 3.13 C | Gaussian Integer | [N-21](#n-21) |
| 3.13 C | 連分数・有理数近似 | [N-22](#n-22) |
| 3.13 C | Pell方程式 | [N-23](#n-23) |
| 3.13 C | 二平方数和の表示 | [N-24](#n-24) |
| 3.14 C | 高速素数和 | [N-25](#n-25) |
| 3.14 C | 一般化されたMin_25 Sieve | [N-26](#n-26) |
| 3.14 C | Dirichlet逆元 | [N-10](#n-10) |
| 3.14 C | 任意乗法的関数の大規模総和 | [N-27](#n-27) |
| 3.14 C | 特殊なmod和 | [N-28](#n-28) |
| 3.15 C | RSK対応 | [C-20](#c-20) |
| 3.15 C | Young Tableaux | [C-21](#c-21) |
| 3.15 C | Hook-Length Formula | [C-22](#c-22) |
| 3.15 C | P-recursive数列 | [Z-25](#z-25) |
| 3.15 C | 高度な組合せ恒等式用の変換 | [C-23](#c-23) |
| 3.16 C | Rader FFT | [Q-20](#q-20) |
| 3.16 C | Bluestein FFT | [Q-21](#q-21) |
| 3.16 C | 高性能64bit NTT | [Q-22](#q-22) |
| 3.16 C | AVX2 NTT | [Q-23](#q-23) |
| 3.16 C | 多変数巡回畳み込み | [Q-25](#q-25) |
| 3.16 C | Multiplicative Convolution | [Q-26](#q-26) |
| 3.17 C | Power Projection | [Q-27](#q-27) |
| 3.17 C | 多項式の根の高速計算 | [Q-28](#q-28) |
| 3.17 C | 高速FPS合成 | [Q-29](#q-29) |
| 3.17 C | 特殊なFPS変換 | [Q-30](#q-30) |
| 3.17 C | 二変数FPS | [Q-31](#q-31) |
| 3.17 C | 多変数FPS | [Q-31](#q-31) |
| 3.17 C | FPSの微分方程式 | [Q-33](#q-33) |
| 3.17 C | 多項式行列の行列式・Prefix Product | [Q-24](#q-24) |
| 3.17 C | Composed Sum/Product | [Q-34](#q-34) |
| 3.17 C | 高速中間積 | [Q-35](#q-35) |
| 3.18 C | Holonomic Sequence | [Q-36](#q-36) |
| 3.18 C | Reeds–Sloane | [Q-37](#q-37) |
| 3.18 C | 特殊な母関数の合成 | [Q-38](#q-38) |
| 3.18 C | 線形漸化式の大量項計算 | [Q-39](#q-39) |
| 3.19 C | SPS Power Projection | [Q-40](#q-40) |
| 3.19 C | Online Set Function Convolution | [Q-41](#q-41) |
| 3.19 C | 特殊な集合関数の変換 | [Q-42](#q-42) |
| 3.20 C | Frobenius Normal Form | [L-20](#l-20) |
| 3.20 C | 行列のHafnian | [L-21](#l-21) |
| 3.20 C | 多項式行列の行列式 | [Q-24](#q-24) |
| 3.20 C | 高速な特性多項式計算 | [L-22](#l-22) |
| 3.21 C | Double-Ended Palindromic Tree | [S-20](#s-20) |
| 3.21 C | Online Z Algorithm | [S-21](#s-21) |
| 3.21 C | Generalized Suffix Automaton | [S-22](#s-22) |
| 3.21 C | 高度な文字列周期問題 | [S-23](#s-23) |
| 3.22 C | Delaunay Triangulation | [H-20](#h-20) |
| 3.22 C | Voronoi Diagram | [H-21](#h-21) |
| 3.22 C | Dynamic Convex Hull | [H-22](#h-22) |
| 3.22 C | Polygon Triangulation | [H-23](#h-23) |
| 3.22 C | Convex Polygon Union Area | [H-17](#h-17) |
| 3.22 C | 三次元幾何 | [H-24](#h-24) |
| 3.23 C | Weighted Matroid Intersection | [O-20](#o-20) |
| 3.23 C | Base Polyhedron | [O-21](#o-21) |
| 3.23 C | Submodular Optimization | [O-22](#o-22) |
| 3.23 C | 特殊な整数計画法 | [O-23](#o-23) |
| 3.24 C | 超限的なゲーム値・高度な組合せゲーム理論 | [B-20](#b-20) |
| 3.25 C | 多点並列探索 | [E-20](#e-20) |
| 3.25 C | 特殊な確率分布の高速サンプリング | [E-21](#e-21) |

## 照合・更新方法

ファイル名だけで未実装と判断せず、公開メソッドと共有先を確認しました。とくにMerge Sort TreeはFC版、動的列はTreap版、一般静的MEXはgrundy_mexで既に満たします。旧コードがあるFastIO・円接線・parserは整備待ちに残しています。

照合時はrg --filesと機能名/公開メソッドの検索を併用し、該当ヘッダ・個別説明・既存検証コードを確認しました。初回監査は文書の追加だったため全アルゴリズムの再テストはせず、ローカル参照先・ID重複・元Cの全件対応・件数を検査しています。前回のB実装の通常/Sanitizer/回帰結果は引継書に記録済みです。

今後は各IDの残る仕様を満たした時点で状態と根拠を更新し、元Cの対応表も同時に維持してください。任意の追加方式や高速化を、既存機能全体が未実装であるという意味にはしません。

2026-10-19のF-01〜F-07追加分は `python3 verify/run_foundation.py` / `--sanitize`、図解モデル・ブラウザ検証を使用します。[検証範囲](basic_utility/foundation.md)。

2026-10-19のF-08〜F-10追加分は `python3 verify/run_foundation_structures.py` / `--sanitize`、5モードの図解モデル・ブラウザ検証を使用します。[検証範囲](foundation_structures/README.md)。

2026-10-19のF-11追加分は `python3 verify/run_big_integer.py` / `--sanitize`、4モードの図解モデル・ブラウザ検証を使用します。[検証範囲・性能](basic_utility/big_integer.md)。

2026-10-19のF-12〜F-17追加分は `python3 verify/run_foundation_tools.py` / `--sanitize`、6モードの図解モデル・ブラウザ検証を使用します。[検証範囲](foundation_tools/README.md)。

2026-10-20のU-01〜U-11・U-20〜U-22は `make verify-utility-search` と14モードの図解検証を使用します。[検証範囲](utility_search/README.md)。

2026-10-20のD-01は `make verify-static-query` と同カテゴリの6モードの図解検証を使用します。[対応範囲・検証](static_query/swag_deque.md)。

2026-10-24のD-02〜D-10は `make verify-data-structure-extensions` と9モードの図解を使用します。[対応範囲・検証](data_structure_extensions/README.md)。D-11〜D-19の登録はありません。
