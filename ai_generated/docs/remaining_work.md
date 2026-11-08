# 未実装・追加仕様の統合一覧

更新日: 2026-10-29。初回照合基準はコミット `9067220` と、この依頼の本文・添付2資料・元の3.1〜3.25のC項目。C++版を対象とし、同じ機能・別名・カテゴリ間の重複を1行へ統合しました。

[実装計画](implementation_plan.md) / [引継書](../HANDOFF.md) / [ドキュメント索引](README.md)

これは初回監査と追加実装の進捗を追跡する一覧です。2026-10-19の「未実装のものを順に」の依頼に従い、先頭のF-01〜F-17を整備しました。[F-01〜F-07](basic_utility/foundation.md) / [F-08〜F-10の対応範囲・検証](foundation_structures/README.md)。[F-11の対応範囲・検証](basic_utility/big_integer.md)。[F-12〜F-17の対応範囲・検証](foundation_tools/README.md)。[V-01〜V-04の開発・検証基盤](development/README.md)も整備しました。[V-05〜V-07の実行・観察ツール](development_runtime/README.md)も整備しました。[V-08〜V-10の出典・検証台帳と共通debug mode](development_records/README.md)も整備しました。[U-01〜U-11・U-20〜U-22の14項目](utility_search/README.md)も整備しました。[D-01（SWAG Deque）](static_query/swag_deque.md)も整備しました。[D-02〜D-10](data_structure_extensions/README.md)も整備しました。[D-20〜D-35の16項目](data_structure_advanced/README.md)も整備しました。今回の指定に従い[T全7項目](tree_extensions/README.md)も整備しました。D-11〜D-19の行はなく、[G-01〜G-15](graph_extensions/README.md)も整備しました。[G残り15項目](graph_advanced/README.md)も整備済みです。[P全14項目](dp_extensions/README.md)も整備済みです。[N全19項目](number_theory_extensions/README.md)も整備済みです。[C全8項目](combinatorics_extensions/README.md)も整備済みです。[Q全31項目](polynomial_extensions/README.md)も整備済みです。[L全11項目](linear_algebra_extensions/README.md)も整備済みです。[S全11項目](string_extensions/README.md)も整備済みです。[H全22項目](geometry_extensions/README.md)も整備済みです。[Y全5項目](numerical_analysis/README.md)も整備済みです。[O全8項目・B-20](optimization_extensions/README.md)も整備済みです。[E全8項目・X全3項目](search_parsing_extensions/README.md)も整備済みです。追跡対象245項目の残作業は0です。初回監査では既存コードと公開API・説明・検証の範囲を確認しました。添付にあるLibrary Checkerの新着区分・他ライブラリの収録状況全体は再監査していません。T追加分の出典と動的木2問題の入出力仕様は[一次資料](tree_extensions/references.md)で別途確認しました。

資料1は「1. 追加対象の区分」で始まる添付、資料2は「調査範囲と凡例」で始まる添付です。本文と2資料の候補を含め、元Cは末尾の全件対応表で追跡します。既存S全体の再整備やPythonへの全移植は別の計画で、この一覧の件数に含めません。

## 件数と判定基準

| 区分 | 件数 | 意味 |
|---|---:|---|
| 未実装 | 0 | 該当する公開機能を確認できない。 |
| 一部対応 | 0 | 基礎/関連機能はあり、記載した追加仕様・汎用化・性能が不足。 |
| 整備待ち | 0 | 旧実装あり。再実装より先に新API/単独include/説明/検証を整える。 |
| 要仕様化 | 0 | 「特殊な」「高度な」等で範囲が決まらず、完成/未実装を一律に断定できない。 |
| 残作業の合計 | 0 | 下表の未完了行。元リストの掲載数とは異なる。 |
| 追加整備済み | 245 | F-01〜F-17、V-01〜V-10、U-01〜U-11・U-20〜U-22、D-01〜D-10・D-20〜D-35、T-01〜T-03・T-20〜T-23、G-01〜G-16・G-20〜G-33、P-01〜P-10・P-20〜P-23、N-01〜N-10・N-20〜N-28、C-01〜C-04・C-20〜C-23、Q全31項目、L全11項目、S全11項目、H全22項目、Y全5項目、O全8項目、B-20、E全8項目、X全3項目。完了行もIDを保持し追跡する。 |
| 追跡対象の合計 | 245 | 残作業0＋追加整備済み245。 |
| 既存対応で除外 | 27 | 後掲の参照表。残作業には重複計上しない。 |

「旧Cはすべて未実装」という前提は採用しません。たとえば元3.15 CのP-recursive数列は3.18 Bで既知漸化式のprefix生成が実装済みです。一方、Holonomicの推定・閉包操作などは別の残作業です。O(n³)の特性多項式・既存の中間積・基本rank/select等も、基礎機能と高度化を分けています。

複数のAPIを1行にまとめたものは共通基盤を持つ作業単位です。例：Resultant/Discriminant、Matroid Union/Partition、線分スイープ/Arrangement。別用途のRange Mex・Range Majority・Range Mode、木の中心・重心、mod 2^64・GF(2^64)は統合しません。追加資料のS/A/B/C案と元リストのランクは混ぜず、出所欄のC3.xだけが元Cを意味します。

## 着手順の候補

1. 基盤型・FastIO整備・共通安全演算・verify CI/stress tester。
2. 尺取り・動的MEX・半分全列挙・Knapsack復元。Segtree境界探索やLIS/LCS復元は既存を使う。
3. 区間辺グラフ・二部被覆の復元・長方形和集合。
4. 素数冪/CRT二項係数・任意mod行列式・SWAG Deque・Gomory–Hu。
5. 数値解析・LP・専門代数・圧縮索引と元C。2026-10-19から一覧掲載順で着手し、F-01〜F-17は整備済み。V-01〜V-04も整備済み。V-05〜V-07も整備済み。V-08〜V-10も整備済み。U-01〜U-11・U-20〜U-22も整備済み。D-01も整備済み。D-02〜D-10・D-20〜D-35も整備済み。今回の指定でT全7項目も整備済み。G-01〜G-15も整備済み。[G残り15項目](graph_advanced/README.md)も整備済みです。[P全14項目](dp_extensions/README.md)も整備済みです。[N全19項目](number_theory_extensions/README.md)も整備済みです。[C全8項目](combinatorics_extensions/README.md)も整備済みです。[Q全31項目](polynomial_extensions/README.md)も整備済みです。[L全11項目](linear_algebra_extensions/README.md)も整備済みです。[S全11項目](string_extensions/README.md)も整備済みです。[H全22項目](geometry_extensions/README.md)も整備済みです。[Y全5項目](numerical_analysis/README.md)も整備済みです。[O全8項目・B-20](optimization_extensions/README.md)も整備済みです。[E全8項目・X全3項目](search_parsing_extensions/README.md)も整備済みです。追跡対象245項目の残作業は0です。

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
| <a id="g-16"></a>G-16 | 高速最小費用流 / Network Simplex・Cost Scaling | 実装あり | Cost Scaling Min-Cost b-Flowとして仕様・復元・計算量を定義。 [実装](../graph_flow/cost_scaling_b_flow.hpp) / [説明・検証](graph_flow/cost_scaling_b_flow.md) / [全体の範囲](graph_advanced/README.md)。 | 追加 |
| <a id="g-20"></a>G-20 | 高速な特殊重み最短路 | 実装あり | 少種類重みの最短路として仕様・復元・計算量を定義。 [実装](../graph_basic/few_weight_shortest_path.hpp) / [説明・検証](graph_basic/few_weight_shortest_path.md) / [全体の範囲](graph_advanced/README.md)。 | C3.7 |
| <a id="g-21"></a>G-21 | グラフ制約付きShortest Nonzero Path | 実装あり | Shortest Nonzero Pathとして仕様・復元・計算量を定義。 [実装](../graph_basic/shortest_nonzero_path.hpp) / [説明・検証](graph_basic/shortest_nonzero_path.md) / [全体の範囲](graph_advanced/README.md)。 | C3.7 |
| <a id="g-22"></a>G-22 | Minimum Diameter Spanning Tree | 実装あり | Minimum Diameter Spanning Treeとして仕様・復元・計算量を定義。 [実装](../graph_basic/minimum_diameter_spanning_tree.hpp) / [説明・検証](graph_basic/minimum_diameter_spanning_tree.md) / [全体の範囲](graph_advanced/README.md)。 | C3.7 |
| <a id="g-23"></a>G-23 | SPQR Tree | 実装あり | SPQR Tree（分離対探索版）として仕様・復元・計算量を定義。 [実装](../graph_connectivity/spqr_tree.hpp) / [説明・検証](graph_connectivity/spqr_tree.md) / [全体の範囲](graph_advanced/README.md)。 | C3.8 |
| <a id="g-24"></a>G-24 | 一般グラフの木幅分解 | 実装あり | 一般グラフの木幅分解として仕様・復元・計算量を定義。 [実装](../graph_connectivity/tree_decomposition.hpp) / [説明・検証](graph_connectivity/tree_decomposition.md) / [全体の範囲](graph_advanced/README.md)。 | C3.8 |
| <a id="g-25"></a>G-25 | 特殊なグラフの標準形 | 実装あり | Cographの標準形として仕様・復元・計算量を定義。 [実装](../graph_connectivity/cograph_canonical_form.hpp) / [説明・検証](graph_connectivity/cograph_canonical_form.md) / [全体の範囲](graph_advanced/README.md)。 | C3.8 |
| <a id="g-26"></a>G-26 | 一般グラフの最大重みマッチング（Blossomの重み付き版） | 実装あり | 一般グラフ最大重みMatchingとして仕様・復元・計算量を定義。 [実装](../graph_flow/general_weighted_matching.hpp) / [説明・検証](graph_flow/general_weighted_matching.md) / [全体の範囲](graph_advanced/README.md)。 | C3.9 |
| <a id="g-27"></a>G-27 | Rank-Maximal Matching | 実装あり | Rank-Maximal Matchingとして仕様・復元・計算量を定義。 [実装](../graph_flow/rank_maximal_matching.hpp) / [説明・検証](graph_flow/rank_maximal_matching.md) / [全体の範囲](graph_advanced/README.md)。 | C3.9 |
| <a id="g-28"></a>G-28 | 特殊な動的フロー | 実装あり | 動的単位容量Max Flowとして仕様・復元・計算量を定義。 [実装](../graph_flow/dynamic_unit_flow.hpp) / [説明・検証](graph_flow/dynamic_unit_flow.md) / [全体の範囲](graph_advanced/README.md)。 | C3.9 |
| <a id="g-29"></a>G-29 | ラベル付き連結グラフ数 | 実装あり | ラベル付き連結グラフ数として仕様・復元・計算量を定義。 [実装](../graph_combinatorics/connected_graph_count.hpp) / [説明・検証](graph_combinatorics/connected_graph_count.md) / [全体の範囲](graph_advanced/README.md)。 | C3.10 |
| <a id="g-30"></a>G-30 | 強連結有向グラフ数 | 実装あり | 強連結有向グラフ数として仕様・復元・計算量を定義。 [実装](../graph_combinatorics/strongly_connected_graph_count.hpp) / [説明・検証](graph_combinatorics/strongly_connected_graph_count.md) / [全体の範囲](graph_advanced/README.md)。 | C3.10 |
| <a id="g-31"></a>G-31 | Euler閉路数（BEST定理） | 実装あり | Euler閉路数（BEST定理）として仕様・復元・計算量を定義。 [実装](../graph_combinatorics/euler_circuit_count.hpp) / [説明・検証](graph_combinatorics/euler_circuit_count.md) / [全体の範囲](graph_advanced/README.md)。 | C3.10 |
| <a id="g-32"></a>G-32 | 一般的なグラフ同型 | 実装あり | 一般グラフの厳密同型判定として仕様・復元・計算量を定義。 [実装](../graph_combinatorics/graph_isomorphism.hpp) / [説明・検証](graph_combinatorics/graph_isomorphism.md) / [全体の範囲](graph_advanced/README.md)。 | C3.10 |
| <a id="g-33"></a>G-33 | 特殊なグラフ分解 | 実装あり | 二端子直並列グラフ分解として仕様・復元・計算量を定義。 [実装](../graph_combinatorics/series_parallel_decomposition.hpp) / [説明・検証](graph_combinatorics/series_parallel_decomposition.md) / [全体の範囲](graph_advanced/README.md)。 | C3.10 |

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
| <a id="p-01"></a>P-01 | Profile / Plug / Frontier DP | 実装あり | 境界の次数と連結成分ラベルを正規化し、指定次数0〜2の連結辺集合と全頂点経路/閉路を数え上げ。辺順序・多重辺・終了条件を定義。 [実装](../dp_optimization/frontier_dp.hpp) / [説明・検証](dp_optimization/frontier_dp.md)。 | 追加 |
| <a id="p-02"></a>P-02 | Bit-parallel LCS | 実装あり | 256種類のバイト列のLCS長。64bit間のshift carry/減算borrow、長さ制限なしの複数word。 [実装](../dp_optimization/bit_parallel_lcs.hpp) / [説明・検証](dp_optimization/bit_parallel_lcs.md)。 | 追加 |
| <a id="p-03"></a>P-03 | Bit-parallel Edit Distance / Myers | 実装あり | 挿入・削除・置換が各1のglobal Levenshtein距離。64bit block間の水平差分と最後の部分word。 [実装](../dp_optimization/myers_edit_distance.hpp) / [説明・検証](dp_optimization/myers_edit_distance.md)。 | 追加 |
| <a id="p-04"></a>P-04 | Exact Cover / Algorithm X / Dancing Links | 実装あり | primary丁度1回・secondary高々1回。Dancing Linksで一解/全解、行ID、上限・callback停止・再利用。 [実装](../dp_optimization/exact_cover.hpp) / [説明・検証](dp_optimization/exact_cover.md)。 | 追加 |
| <a id="p-05"></a>P-05 | A* | 実装あり | 非負整数費用の暗黙状態、許容的な非整合hでも再展開、不変の親履歴と操作IDの復元。 [実装](../dp_optimization/a_star.hpp) / [説明・検証](dp_optimization/a_star.md)。 | 追加 |
| <a id="p-06"></a>P-06 | IDA* | 実装あり | 最小の閾値超過で反復。経路上の循環除去・明示stack、費用と操作列の復元、有限状態で到達不能も終了。 [実装](../dp_optimization/ida_star.hpp) / [説明・検証](dp_optimization/ida_star.md)。 | 追加 |
| <a id="p-07"></a>P-07 | 双方向状態空間探索 | 実装あり | 単位費用の有向状態空間。逆遷移callback、小さい側の層全体を展開、最短合流と順向き操作ID復元。 [実装](../dp_optimization/bidirectional_search.hpp) / [説明・検証](dp_optimization/bidirectional_search.md)。 | 追加 |
| <a id="p-08"></a>P-08 | 汎用SAT Solver | 実装あり | 一般CNFのDPLLと2 watched literals。単位伝播、明示decision stack、仮定と追加節、SAT代入/UNSAT。 [実装](../dp_optimization/sat_solver.hpp) / [説明・検証](dp_optimization/sat_solver.md)。 | 追加 |
| <a id="p-09"></a>P-09 | ナップサック・部分和の復元 | 実装あり | 0-1/完全/個数制限の段階別採用個数をO(nW)で復元。Bitsetは新規到達bitの親だけを保存。 [実装](../dp_optimization/knapsack_reconstruction.hpp) / [説明・検証](dp_optimization/knapsack_reconstruction.md)。 | 追加 |
| <a id="p-10"></a>P-10 | 最適解の個数を返すDP API | 実装あり | LISの添字列・LCSの一致添字pair列・0-1品物ID集合の最適解数。空解1、重さ0同点、BigInt/合成数mod。 [実装](../dp_optimization/optimal_solution_count.hpp) / [説明・検証](dp_optimization/optimal_solution_count.md)。 | 追加 |
| <a id="p-20"></a>P-20 | 高度なMonge行列積 | 実装あり | 任意の有限AとMonge Bの長方形min-plus積。SMAWKでO(rows*(inner+cols))、最左argminを復元。 [実装](../dp_optimization/monge_matrix_product.hpp) / [説明・検証](dp_optimization/monge_matrix_product.md)。 | C3.12 |
| <a id="p-21"></a>P-21 | 複雑なSlope Trick | 実装あり | BigIntの重み付きhingeをmapで圧縮。巨大重みを展開せず、prefix/suffix・区間shift・merge・反転・評価。 [実装](../dp_optimization/weighted_slope_trick.hpp) / [説明・検証](dp_optimization/weighted_slope_trick.md)。 | C3.12 |
| <a id="p-22"></a>P-22 | 動的な最適化DP | 実装あり | 固定状態数の遷移行列列をmin-plus segment treeで更新。区間積O(k³ log n)、負費用・到達不能・状態列復元。 [実装](../dp_optimization/dynamic_finite_state_dp.hpp) / [説明・検証](dp_optimization/dynamic_finite_state_dp.md)。 | C3.12 |
| <a id="p-23"></a>P-23 | 特殊な高速ナップサック | 実装あり | 重さD種類の0-1 Knapsack。価値順prefixの凹性と単調argmaxでO(n log n + D W log(W+1))。 [実装](../dp_optimization/small_weight_knapsack.hpp) / [説明・検証](dp_optimization/small_weight_knapsack.md)。 | C3.12 |

## 数論（3.13〜3.14）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="n-01"></a>N-01 | 区間篩 / Segmented Sieve | 実装あり | uint64半開区間の素数・素因数分解。幅/平方根の予算、0の分解を区別。 [実装](../number_theory_basic/segmented_sieve.hpp) / [説明・検証](number_theory_basic/segmented_sieve.md)。 | 追加 |
| <a id="n-02"></a>N-02 | 乗法位数・Carmichael関数 | 実装あり | uint64のλと可逆元の最小正位数。Pollard–Rho共有、非unitはnullopt。 [実装](../number_theory_basic/multiplicative_order.hpp) / [説明・検証](number_theory_basic/multiplicative_order.md)。 | 追加 |
| <a id="n-03"></a>N-03 | Min of Mod of Linear | 実装あり | BigIntの符号付き区間、最小剰余と最初の添字をO(log² m)で取得。 [実装](../number_theory_basic/min_mod_linear.hpp) / [説明・検証](number_theory_basic/min_mod_linear.md)。 | 追加 |
| <a id="n-04"></a>N-04 | 重み付き/多項式Floor Sum・万能Euclid | 実装あり | BigIntの混合momentと非可換モノイドのEuclid操作列。負係数にも対応。 [実装](../number_theory_basic/floor_sum_polynomial.hpp) / [説明・検証](number_theory_basic/floor_sum_polynomial.md)。 | 追加 |
| <a id="n-05"></a>N-05 | Batch Inversion | 実装あり | prefix積と1回の逆元。可換環のcallback版とuint64任意mod版、非unitはnullopt。 [実装](../number_theory_basic/batch_inversion.hpp) / [説明・検証](number_theory_basic/batch_inversion.md)。 | 追加 |
| <a id="n-06"></a>N-06 | 素数冪/CRTによる大引数の任意mod二項係数 | 実装あり | 固定法の素数冪表とCRT。uint64の巨大n/k、多数query、表サイズ予算を明示。 [実装](../combinatorics/prime_power_binomial.hpp) / [説明・検証](combinatorics/prime_power_binomial.md)。 | 追加 |
| <a id="n-07"></a>N-07 | 巨大なnの階乗 mod p | 実装あり | 実行時素数2..2×10^9。Wilsonと積/剰余木・多点評価で平方根規模の階乗。 [実装](../combinatorics/large_factorial.hpp) / [説明・検証](combinatorics/large_factorial.md)。 | 追加 |
| <a id="n-08"></a>N-08 | 法固定・前計算再利用の離散対数 | 実装あり | 底/法固定のbaby tableとgcd縮小列を共有。非互いに素・uint64・最小指数。 [実装](../number_theory_basic/discrete_log_table.hpp) / [説明・検証](number_theory_basic/discrete_log_table.md)。 | 追加 |
| <a id="n-09"></a>N-09 | 商列挙prefix表上のDirichlet積 | 実装あり | QuotientPrefixと双曲線恒等式。全商でO(N^(3/4))環演算、O(√N)領域。 [実装](../number_theory_convolution/dirichlet_prefix_product.hpp) / [説明・検証](number_theory_convolution/dirichlet_prefix_product.md)。 | 追加 |
| <a id="n-10"></a>N-10 | Dirichlet逆元 | 実装あり | 配列O(n log n)と商prefix O(N^(3/4))の双方。f(1)の逆元callback、非unit拒否。 [実装](../number_theory_convolution/dirichlet_inverse.hpp) / [説明・検証](number_theory_convolution/dirichlet_inverse.md)。 | 追加, C3.14 |
| <a id="n-20"></a>N-20 | mod k乗根 | 実装あり | 素数法のk乗根。原始根/BSGS/線形合同式から全根を圧縮、列挙予算あり。 [実装](../number_theory_basic/mod_kth_root.hpp) / [説明・検証](number_theory_basic/mod_kth_root.md)。 | C3.13 |
| <a id="n-21"></a>N-21 | Gaussian Integer | 実装あり | BigIntの実虚成分、厳密な最近整数除算・norm・共役・正規化gcd。 [実装](../number_theory_basic/gaussian_integer.hpp) / [説明・検証](number_theory_basic/gaussian_integer.md)。 | C3.13 |
| <a id="n-22"></a>N-22 | 連分数・有理数近似 | 実装あり | 正則連分数、全収束分数、分母制限の最小絶対誤差近似。同点規則を明示。 [実装](../number_theory_basic/continued_fraction.hpp) / [説明・検証](number_theory_basic/continued_fraction.md)。 | C3.13 |
| <a id="n-23"></a>N-23 | Pell方程式 | 実装あり | 正/負Pellの最小正解と解の累乗。uint64の非平方D、BigInt解、反復予算。 [実装](../number_theory_basic/pell_equation.hpp) / [説明・検証](number_theory_basic/pell_equation.md)。 | C3.13 |
| <a id="n-24"></a>N-24 | 二平方数和の表示 | 実装あり | uint64の全非負unordered表示。Gaussian因子の指数分配、候補予算、重複除去。 [実装](../number_theory_basic/sum_two_squares.hpp) / [説明・検証](number_theory_basic/sum_two_squares.md)。 | C3.13 |
| <a id="n-25"></a>N-25 | 高速素数和 | 実装あり | BigIntの素数和と全商表。完全乗法的重み・prefix callbackの素数篩も公開。 [実装](../number_theory_convolution/prime_sum.hpp) / [説明・検証](number_theory_convolution/prime_sum.md)。 | C3.14 |
| <a id="n-26"></a>N-26 | 一般化されたMin_25 Sieve | 実装あり | 素数上の多項式と任意の素数冪callback。合成数modも可、全商の総和を復元。 [実装](../number_theory_convolution/min25_sieve.hpp) / [説明・検証](number_theory_convolution/min25_sieve.md)。 | C3.14 |
| <a id="n-27"></a>N-27 | 任意乗法的関数の大規模総和 | 実装あり | 任意の素数prefix表とf(p^e)から大規模総和。素数prefix構築の条件/費用は別。 [実装](../number_theory_convolution/multiplicative_summatory.hpp) / [説明・検証](number_theory_convolution/multiplicative_summatory.md)。 | C3.14 |
| <a id="n-28"></a>N-28 | 特殊なmod和 | 実装あり | Σ i^p·((a*i+b) mod m)^qの厳密和と任意法への出力。N-04のmomentを共有。 [実装](../number_theory_basic/mod_sum_polynomial.hpp) / [説明・検証](number_theory_basic/mod_sum_polynomial.md)。 | C3.14 |

## 組合せ・数え上げ（3.15）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="c-01"></a>C-01 | q-二項係数・q-階乗 | 実装あり | 素数法q-Lucasと消失周期、q=0/1を含む。 [API・検証](combinatorics/q_binomial.md)。 | 追加 |
| <a id="c-02"></a>C-02 | 上下境界付き単調列/格子路の数え上げ | 実装あり | 正規化した上下境界を長方形分割＋畳み込みで転送。 [API・検証](combinatorics/bounded_monotone.md)。 | 追加 |
| <a id="c-03"></a>C-03 | 小さい素数法・巨大nのStirling数 | 実装あり | O(p²)前計算、uint64全域の第1/第2種をLucasで評価。 [API・検証](combinatorics/stirling_small_prime.md)。 | 追加 |
| <a id="c-04"></a>C-04 | ラベルなし木の数え上げ | 実装あり | BigIntの根付き/無根同型類数。 [API・検証](combinatorics/unlabeled_trees.md)。 | 追加 |
| <a id="c-20"></a>C-20 | RSK対応 | 実装あり | word/biword/行列の正逆対応。 [API・検証](combinatorics/rsk.md)。 | C3.15 |
| <a id="c-21"></a>C-21 | Young Tableaux | 実装あり | 盤の検査・共役/転置・降下・callback列挙。 [API・検証](combinatorics/young_tableaux.md)。 | C3.15 |
| <a id="c-22"></a>C-22 | Hook-Length Formula | 実装あり | 厳密値・任意uint64法・半標準hook-content。 [API・検証](combinatorics/hook_length.md)。 | C3.15 |
| <a id="c-23"></a>C-23 | 高度な組合せ恒等式用の変換 | 実装あり | 承認仕様の高速Lah/Euler変換と逆変換。 [API・検証](combinatorics/combinatorial_transforms.md)。 | C3.15 |

## 畳み込み・多項式・母関数（3.16〜3.19）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="q-01"></a>Q-01 | 多項式拡張GCD・mod逆元 | 追加整備済み | [API・条件・計算量・検証](fps/polynomial_extended_gcd.md) / [図解](../visualizations/polynomial_extensions/index.html#extended_gcd)。a*u+b*v=gcdを保ちながらEuclid法を進める。定数倍を最後に正規化する。 | 追加 |
| <a id="q-02"></a>Q-02 | Polynomial Resultant / Discriminant | 追加整備済み | [API・条件・計算量・検証](fps/polynomial_resultant.md) / [図解](../visualizations/polynomial_extensions/index.html#resultant)。剰余への置換時に先頭係数の冪と交換符号を掛ける。判別式は(-1)^(d(d-1)/2) Res(f,f')/lc(f)。 | 追加 |
| <a id="q-03"></a>Q-03 | 有限体上の多項式因数分解 | 追加整備済み | [API・条件・計算量・検証](fps/polynomial_factorization.md) / [図解](../visualizations/polynomial_extensions/index.html#factorization)。微分とのGCDで重複を分け、微分0ならp乗根へ再帰。Frobeniusで次数別分解し、奇標数は二次指標、標数2はtraceで分割する。 | 追加 |
| <a id="q-04"></a>Q-04 | mod 2^64の畳み込み | 追加整備済み | [API・条件・計算量・検証](convolution/convolution_u64.md) / [図解](../visualizations/polynomial_extensions/index.html#wrap_convolution)。16bitの4分割。次数和が4未満のlimb対だけを集約し、2素数CRTで整数復元後に64bitへ折り返す。容量内では各対角和がCRTの積未満。 | 追加 |
| <a id="q-05"></a>Q-05 | GF(2^64)上の畳み込み | 追加整備済み | [API・条件・計算量・検証](convolution/convolution_nimber.md) / [図解](../visualizations/polynomial_extensions/index.html#nimber_convolution)。既存Nimberと環上Karatsubaを共有し、公開入口を整数の係数表現へ接続する。 | 追加 |
| <a id="q-06"></a>Q-06 | 転置原理 / Tellegenの公開部品 | 追加整備済み | [API・条件・計算量・検証](convolution/transposed_convolution.md) / [図解](../visualizations/polynomial_extensions/index.html#transpose)。切り詰めToeplitz作用と、その転置の相関計算を、周波数添字の反転で実装。内積〈Ax,w〉=〈x,Aᵀw〉を検証する。 | 追加 |
| <a id="q-07"></a>Q-07 | NTT Doubling | 追加整備済み | [API・条件・計算量・検証](convolution/ntt_doubling.md) / [図解](../visualizations/polynomial_extensions/index.html#doubling)。偶数周波数は既知の変換値、奇数周波数だけ逆変換・twist・再変換で補う。 | 追加 |
| <a id="q-08"></a>Q-08 | Sharp P Subset Sum | 追加整備済み | [API・条件・計算量・検証](fps/sharp_p_subset_sum.md) / [図解](../visualizations/polynomial_extensions/index.html#subset_sum)。重み頻度を作り、Σ log(1+x^s)の係数を約数倍の走査で集計してexpする。 | 追加 |
| <a id="q-09"></a>Q-09 | 多項式の不定和分 / Prefix Sum of Polynomial | 追加整備済み | [API・条件・計算量・検証](fps/polynomial_prefix_sum.md) / [図解](../visualizations/polynomial_extensions/index.html#prefix_sum)。Pascal係数による差分作用は次数を1下げる三角行列。高次から解いて定数0を指定する。 | 追加 |
| <a id="q-20"></a>Q-20 | Rader FFT | 追加整備済み | [API・条件・計算量・検証](convolution/rader_fft.md) / [図解](../visualizations/polynomial_extensions/index.html#rader)。非零添字を原始根の冪に並べ替え、巡回畳み込みへ帰着。 | C3.16 |
| <a id="q-21"></a>Q-21 | Bluestein FFT | 追加整備済み | [API・条件・計算量・検証](convolution/bluestein_fft.md) / [図解](../visualizations/polynomial_extensions/index.html#bluestein)。2ij=i²+j²-(i-j)²を用いてchirpの畳み込みへ変換する。 | C3.16 |
| <a id="q-22"></a>Q-22 | 高性能64bit NTT | 追加整備済み | [API・条件・計算量・検証](convolution/ntt64.md) / [図解](../visualizations/polynomial_extensions/index.html#ntt64)。Goldilocks素数専用reducerと段ごとの単位根cache。任意の64bit法には対応しない。 | C3.16 |
| <a id="q-23"></a>Q-23 | AVX2 NTT | 追加整備済み | [API・条件・計算量・検証](convolution/ntt_avx2.md) / [図解](../visualizations/polynomial_extensions/index.html#avx2)。target属性とCPU検出で命令対応を確認。force_scalarは比較検証や環境選択用。 | C3.16 |
| <a id="q-25"></a>Q-25 | 多変数巡回畳み込み | 追加整備済み | [API・条件・計算量・検証](convolution/multidimensional_cyclic_convolution.md) / [図解](../visualizations/polynomial_extensions/index.html#cyclic_tensor)。既存tensor通常積を共有し、各軸を指定周期で還元。任意の軸長を扱う。 | C3.16 |
| <a id="q-26"></a>Q-26 | Multiplicative Convolution | 追加整備済み | [API・条件・計算量・検証](convolution/multiplicative_convolution.md) / [図解](../visualizations/polynomial_extensions/index.html#multiplicative)。非零添字を原始根の離散対数順に並べ、指数の巡回積を取る。添字0の寄与は別途集計。 | C3.16 |
| <a id="q-27"></a>Q-27 | Power Projection | 追加整備済み | [API・条件・計算量・検証](fps/power_projection.md) / [図解](../visualizations/polynomial_extensions/index.html#projection)。[x^(n−1)] reverse(weights)/(1−yg(x))を二変数Bostan–Moriで縮約。x次数を半分、y次数を倍にする。 | C3.17 |
| <a id="q-28"></a>Q-28 | 多項式の根の高速計算 | 追加整備済み | [API・条件・計算量・検証](fps/polynomial_roots.md) / [図解](../visualizations/polynomial_extensions/index.html#roots)。Frobeniusの固定点だけをGCDで抽出し、その一次因子のみを分割。非線形既約因子の全分解を避ける。 | C3.17 |
| <a id="q-29"></a>Q-29 | 高速FPS合成 | 追加整備済み | [API・条件・計算量・検証](fps/fast_composition.md) / [図解](../visualizations/polynomial_extensions/index.html#composition)。Power Projectionの各線形作用を逆順に転置するKinoshita–Li型の構成。既存BSGSを呼び直していない。 | C3.17 |
| <a id="q-30"></a>Q-30 | 特殊なFPS変換 | 追加整備済み | [API・条件・計算量・検証](fps/fractional_linear_transform.md) / [図解](../visualizations/polynomial_extensions/index.html#fractional)。`[x^k](x/(1−cx))^j=C(k−1,j−1)c^(k−j)`。Pascal行を更新する。 | C3.17 |
| <a id="q-31"></a>Q-31 | 多変数FPS（二変数を含む） | 追加整備済み | [API・条件・計算量・検証](fps/multivariate_fps.md) / [図解](../visualizations/polynomial_extensions/index.html#multivariate)。Euler微分Σx_i∂_iと次数による積分、nilpotent ideal上のNewton倍増。通常tensor積から矩形領域を抽出。 | C3.17 |
| <a id="q-33"></a>Q-33 | FPSの微分方程式 | 追加整備済み | [API・条件・計算量・検証](fps/differential_equation.md) / [図解](../visualizations/polynomial_extensions/index.html#ode)。積分因子による線形解とNewton線形化。補正δ'=F_yδ+F−y'、δ(0)=0を倍増精度で解く。 | C3.17 |
| <a id="q-24"></a>Q-24 | 多項式行列の行列式・Prefix Product | 追加整備済み | [API・条件・計算量・検証](fps/polynomial_matrix.md) / [図解](../visualizations/polynomial_extensions/index.html#polynomial_matrix)。係数の多項式そのものを保持した分数なし消去と、連続評価点のbaby-step/giant-step積。単なる一地点評価のAPIではない。 | C3.17, C3.20 |
| <a id="q-34"></a>Q-34 | Composed Sum/Product | 追加整備済み | [API・条件・計算量・検証](fps/composed_polynomial.md) / [図解](../visualizations/polynomial_extensions/index.html#composed)。逆順多項式の対数微分から根の冪和を取得。和は二項畳み込み、積は冪和の点ごとの積。expで多項式へ戻す。 | C3.17 |
| <a id="q-35"></a>Q-35 | 高速中間積 | 追加整備済み | [API・条件・計算量・検証](convolution/fast_middle_product.md) / [図解](../visualizations/polynomial_extensions/index.html#middle)。反転bとの巡回積で折返しが起こるのは採用範囲より前だけ。必要な中央区間は汚染されない。 | C3.17 |
| <a id="q-36"></a>Q-36 | Holonomic Sequence | 追加整備済み | [API・条件・計算量・検証](linear_recurrence/holonomic.md) / [図解](../visualizations/polynomial_extensions/index.html#holonomic)。線形方程式の零空間から係数を推定。大引数は分母を分離したcompanion行列とその積を使用。一般の閉包演算はこの具体化には含めない。 | C3.18 |
| <a id="q-37"></a>Q-37 | Reeds–Sloane | 追加整備済み | [API・条件・計算量・検証](linear_recurrence/reeds_sloane.md) / [図解](../visualizations/polynomial_extensions/index.html#reeds_sloane)。原論文Section IVのdiscrepancyを単元とp進付値へ分解する手順。各素数冪の最小関係をCRTで合成。lengthは分母次数だけでなく分子次数+1も含む。 | C3.18 |
| <a id="q-38"></a>Q-38 | 特殊な母関数の合成 | 追加整備済み | [API・条件・計算量・検証](linear_recurrence/combinatorial_composition.md) / [図解](../visualizations/polynomial_extensions/index.html#labelled)。SET=exp(f)、SEQ=1/(1−f)、CYC=−log(1−f)。 | C3.18 |
| <a id="q-39"></a>Q-39 | 線形漸化式の大量項計算 | 追加整備済み | [API・条件・計算量・検証](linear_recurrence/recurrence_slice.md) / [図解](../visualizations/polynomial_extensions/index.html#recurrence_slice)。一度だけKitamasaで開始位置へ移り、最初のd項から分子を作って1/Qを展開する。各項ごとの大引数計算を避ける。 | C3.18 |
| <a id="q-40"></a>Q-40 | SPS Power Projection | 追加整備済み | [API・条件・計算量・検証](set_function/sps_power_projection.md) / [図解](../visualizations/polynomial_extensions/index.html#sps_projection)。空集合の定数を分離し、nilpotent部分のb乗までの内積を計算。二項変換で全kへ戻す。 | C3.19 |
| <a id="q-41"></a>Q-41 | Online Set Function Convolution | 追加整備済み | [API・条件・計算量・検証](set_function/online_subset_convolution.md) / [図解](../visualizations/polynomial_extensions/index.html#online_subset)。最上位bitで低半分と高半分に分割。高半分同士はdisjointでないため寄与せず、既知kernelと新入力の2つのonline積へ帰着。 | C3.19 |
| <a id="q-42"></a>Q-42 | 特殊な集合関数の変換 | 追加整備済み | [API・条件・計算量・検証](set_function/intersection_convolution.md) / [図解](../visualizations/polynomial_extensions/index.html#intersection)。rank別zeta表の多項式積をMöbius反転し、rank総和−|S|を交差サイズとして読む。k=0は通常subset convolution。 | C3.19 |

## 線形代数・専門的代数（3.20）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="l-01"></a>L-01 | 任意/合成数modの行列式 | 追加整備済み | [API・条件・検証](linear_algebra/determinant_any_mod.md) / [図解](../visualizations/linear_algebra_extensions/index.html#det_mod)。代表元のEuclid除算で行を消去する。行操作の係数は整数なので、法の非unitを逆元にしない。 | 追加 |
| <a id="l-02"></a>L-02 | 整数行列の厳密行列式 | 追加整備済み | [API・条件・検証](linear_algebra/determinant_integer.md) / [図解](../visualizations/linear_algebra_extensions/index.html#det_integer)。Bareissの分数なし消去。前段pivotによる除算が割り切れることを毎回検査する。 | 追加 |
| <a id="l-03"></a>L-03 | 半環行列・Min-Plus/Max-Plus/Booleanの共通API | 追加整備済み | [API・条件・検証](linear_algebra/semiring_matrix.md) / [図解](../visualizations/linear_algebra_extensions/index.html#semiring)。零元と単位元を明示し、加算・乗算を置き換える。Tropicalでは辺の重みを足し、経路の候補からmin/maxを選ぶ。 | 追加 |
| <a id="l-04"></a>L-04 | Pfaffian | 追加整備済み | [API・条件・検証](linear_algebra/pfaffian.md) / [図解](../visualizations/linear_algebra_extensions/index.html#pfaffian)。行・列を同時に交換し、2×2の交代pivotを取り出す。交換の符号とSchur補元を更新する。 | 追加 |
| <a id="l-05"></a>L-05 | Permanent | 追加整備済み | [API・条件・検証](linear_algebra/permanent.md) / [図解](../visualizations/linear_algebra_extensions/index.html#permanent)。Ryserの包除原理。Gray codeで列集合を一つずつ更新し、行和の積を符号付きで集約する。 | 追加 |
| <a id="l-06"></a>L-06 | Smith Normal Form・整数/一般合同線形系 | 追加整備済み | [API・条件・検証](linear_algebra/smith_normal_form.md) / [図解](../visualizations/linear_algebra_extensions/index.html#smith)。pivotが残り全成分を割り切るまで行・列Euclid変形する。bをUで変換し、対角上の整数除算またはgcd条件を解いてVで戻す。 | 追加 |
| <a id="l-07"></a>L-07 | Hermite Normal Form | 追加整備済み | [API・条件・検証](linear_algebra/hermite_normal_form.md) / [図解](../visualizations/linear_algebra_extensions/index.html#hermite)。列を左から見て整数Euclid法でpivotを作り、正符号に揃える。上側はfloor除算で標準剰余へ減らす。 | 追加 |
| <a id="l-08"></a>L-08 | 汎用拡大有限体・GF(2)多項式演算 | 追加整備済み | [API・条件・検証](linear_algebra/gf2_polynomial.md) / [図解](../visualizations/linear_algebra_extensions/index.html#gf2) / [API・条件・検証](linear_algebra/finite_extension_field.md) / [図解](../visualizations/linear_algebra_extensions/index.html#extension)。F_p[x]/(f)の剰余類。fの既約性をFrobeniusとgcdで確かめ、逆元は多項式拡張Euclid法で求める。 | 追加 |
| <a id="l-20"></a>L-20 | Frobenius Normal Form | 追加整備済み | [API・条件・検証](linear_algebra/frobenius_normal_form.md) / [図解](../visualizations/linear_algebra_extensions/index.html#frobenius)。xI−Aを多項式Smith形へ変換する。逆の行変換で得た加群の生成元にAを繰り返し作用させ、実際の相似基底を構成する。 | C3.20 |
| <a id="l-21"></a>L-21 | 行列のHafnian | 追加整備済み | [API・条件・検証](linear_algebra/hafnian.md) / [図解](../visualizations/linear_algebra_extensions/index.html#hafnian)。未使用頂点集合の最小頂点を、その集合の他の各頂点と組にするDP。各完全マッチングを一度ずつ数える。 | C3.20 |
| <a id="l-22"></a>L-22 | 高速な特性多項式計算 | 追加整備済み | [API・条件・検証](linear_algebra/characteristic_polynomial_fast.md) / [図解](../visualizations/linear_algebra_extensions/index.html#charpoly)。次数nの射影が得られる場合のStrassen/Krylov高速経路はO(n^log₂7 log n)。固定試行数で得られなければHessenbergに戻り、一般の最悪時間はO(n³)。 | C3.20 |

## 文字列・圧縮索引（3.21）

[S全11項目の対応範囲・検証](string_extensions/README.md)。S-23は静的区間の周期判定・最小反復周期・反復回数へ具体化しました。S-08〜S-19の行はありません。

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="s-01"></a>S-01 | 部分列オートマトン・異なる部分列数 | 追加整備済み | [API・条件・検証](string_algorithm/subsequence_automaton.md) / [図解](../visualizations/string_extensions/index.html#subsequence)。各byteの次の出現位置へ進む。同じ文字の前回追加前の個数を記録し、重複した部分列を2倍した総数から引く。 | 追加 |
| <a id="s-02"></a>S-02 | Lempel–Ziv（LZ77）分解 | 追加整備済み | [API・条件・検証](string_algorithm/lempel_ziv.md) / [図解](../visualizations/string_extensions/index.html#lz77)。既出suffixの辞書順の直前・直後だけで最長一致長を求め、greedyに因子へする。 | 追加 |
| <a id="s-03"></a>S-03 | BWT・逆BWT | 追加整備済み | [API・条件・検証](string_algorithm/burrows_wheeler.md) / [図解](../visualizations/string_extensions/index.html#bwt)。終端付きsuffixを並べ、直前の文字を取り出す。逆変換では同じ文字の出現順位を保存するLF対応を辿る。 | 追加 |
| <a id="s-04"></a>S-04 | FM-index | 追加整備済み | [API・条件・検証](string_algorithm/fm_index.md) / [図解](../visualizations/string_extensions/index.html#fm_index)。patternを後ろから読み、C[c]+rank_cで候補区間を狭める。位置はLFを辿ってテキスト座標で間引いたSA標本へ戻る。 | 追加 |
| <a id="s-05"></a>S-05 | Prefix-Substring / Semi-local LCS | 追加整備済み | [API・条件・検証](string_algorithm/semi_local_lcs.md) / [図解](../visualizations/string_extensions/index.html#semi_local)。一致セルまたは左strandの開始labelが上より大きいセルでstrandを交換する。終点順の開始labelを範囲計数し、区間長から横切るstrand数を引く。 | 追加 |
| <a id="s-06"></a>S-06 | Border Tree・全周期列挙 | 追加整備済み | [API・条件・検証](string_algorithm/border_tree.md) / [図解](../visualizations/string_extensions/index.html#borders)。prefix functionの失敗リンクを木にする。全border bとperiod n−bの対応を使い、子から親へ出現数を集計する。 | 追加 |
| <a id="s-07"></a>S-07 | SAMのk番目・全出現位置 | 追加整備済み | [API・条件・検証](string_algorithm/suffix_automaton_utilities.md) / [図解](../visualizations/string_extensions/index.html#sam_utilities)。遷移DAGの先の経路数で辞書順の枝を飛ばす。suffix link木のEuler区間に含まれる実prefixの終点を取り出す。 | 追加 |
| <a id="s-20"></a>S-20 | 両端回文木 | 追加整備済み | [API・条件・検証](string_algorithm/double_ended_eertree.md) / [図解](../visualizations/string_extensions/index.html#deque_eertree)。最大回文に覆われない出現の端（surface）を記録する。最長回文のsuffix linkと間接出現数で追加・削除を更新し、persistentなbyte辞書でsuffix探索を短縮する。 | C3.21 |
| <a id="s-21"></a>S-21 | Online Z | 追加整備済み | [API・条件・検証](string_algorithm/online_z.md) / [図解](../visualizations/string_extensions/index.html#online_z)。prefix長i+kのborderに長さkが存在することと、suffix iがprefix kに一致することは同値。オンラインborder木のancestor判定を使ってkを二分探索する。 | C3.21 |
| <a id="s-22"></a>S-22 | Generalized Suffix Automaton | 追加整備済み | [API・条件・検証](string_algorithm/generalized_suffix_automaton.md) / [図解](../visualizations/string_extensions/index.html#generalized_sam)。私有separatorで文書を区切ったSAMを作り、byte遷移だけのDAGとsuffix link木の索引を共有する。出現位置を文書座標に戻す。 | C3.21 |
| <a id="s-23"></a>S-23 | 区間の周期・反復回数 | 追加整備済み | [API・条件・検証](string_algorithm/substring_period.md) / [図解](../visualizations/string_extensions/index.html#period_queries)。SA/LCPの厳密LCEでずらした2区間を比較する。区間長を素因数分解し、周期を各素因数で割っても一致する間だけ縮める。 | C3.21 |

## 計算幾何・平面グラフ（3.6・3.22）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="h-01"></a>H-01 | 長方形の和集合面積 | 追加整備済み | x方向に走査し、圧縮したy座標の被覆長を積分する。 座標はint64全域、戻り値はBigInt。閉矩形で、幅または高さ0は面積・周長とも寄与0。逆順座標は拒否。 [実装](../computational_geometry/rectangle_union.hpp) / [説明・検証](computational_geometry/rectangle_area.md)。 | 追加 |
| <a id="h-02"></a>H-02 | 長方形の和集合周長 | 追加整備済み | 被覆の連結区間数から水平境界を、イベントでの被覆長変化から垂直境界を数える。 面積と同じ入力。同じxでは追加を削除より先に処理し、接する辺を内部境界として消す。 [実装](../computational_geometry/rectangle_union.hpp) / [説明・検証](computational_geometry/rectangle_perimeter.md)。 | 追加 |
| <a id="h-03"></a>H-03 | 円の接線の新API整備 | 追加整備済み | 2円の半径差・半径和と中心間距離から接線の法線を求め、同じ直線を統合する。 線は単位normal・offset（normal·x=offset）と両円の接点を返す。接点が一致しても線を保持。同一点円も無限本。点円同士の異なる2点は1本。 [実装](../computational_geometry/circle_tangents.hpp) / [説明・検証](computational_geometry/circle_tangents.md)。 | 追加 |
| <a id="h-04"></a>H-04 | 三角形の外接円・内接円等の新API整備 | 追加整備済み | 外心は2本の垂直二等分線、内心は辺長の重み付き平均、垂心はA+B+C−2O。 optional。Toleranceで面積0と判定した三角形はnullopt。中心一式は外接円・内接円・重心・垂心を保持。MECと外接円の計算を共有。 [実装](../computational_geometry/triangle_centers.hpp) / [説明・検証](computational_geometry/triangle_centers.md)。 | 追加 |
| <a id="h-05"></a>H-05 | 多角形の重心の新API整備 | 追加整備済み | 有向辺ごとの外積で、原点との三角形の面積と重心を加重する。 単純な境界ring。CW/CCW可。面積0はnullopt。整数入力も結果は任意精度有理点。浮動版は先頭点を原点にして和を取る。 [実装](../computational_geometry/polygon_centroid.hpp) / [説明・検証](computational_geometry/polygon_centroid.md)。 | 追加 |
| <a id="h-06"></a>H-06 | 円と多角形の共通部分面積 | 追加整備済み | 円内の部分は三角形の外積、円外はatan2による扇形の境界積分に置き換える。 単純多角形、CW/CCW可、点円・面積0の領域は0。辺を円との交点で分割し、接触も扱う。穴はringごとの符号付き差を呼出側で合成する。 [実装](../computational_geometry/circle_area.hpp) / [説明・検証](computational_geometry/circle_polygon_area.md)。 | 追加 |
| <a id="h-07"></a>H-07 | 2円の共通部分面積の新API整備 | 追加整備済み | 重なる2つの円弧から、扇形と三角形の差を足す。 包含・内外接・同一円・点円を処理し、非負面積を返す。有限の尺度で使用する。 [実装](../computational_geometry/circle_area.hpp) / [説明・検証](computational_geometry/circle_intersection_area.md)。 | 追加 |
| <a id="h-08"></a>H-08 | 円の和集合面積 | 追加整備済み | 他の円に覆われる角度区間を統合し、露出する円弧の境界積分を足す。 点円は寄与0、完全に同じ円は一度だけ数える。包含・交差・接触を扱う。分類は実際のlong double比較で、Toleranceによる近似同一視はしない。 [実装](../computational_geometry/circle_area.hpp) / [説明・検証](computational_geometry/circle_union_area.md)。 | 追加 |
| <a id="h-09"></a>H-09 | 平面埋め込みの面列挙・双対グラフ | 追加整備済み | 偏角順のhalf-edgeを左側の面に沿って歩き、負向き境界を包含する最小の正向き境界へ所属させる。 直線辺の交差は分割済み、同一座標頂点・自己辺・重複辺は禁止。half_edgesにfrom/to/twin/next/face。faces[0]が唯一の外面。穴・橋・孤立点・非連結成分を同じ面へまとめる。双対は橋のloopと平行辺を保持。外面のareaは0。 [実装](../computational_geometry/planar_embedding.hpp) / [説明・検証](computational_geometry/planar_embedding.md)。 | 追加 |
| <a id="h-10"></a>H-10 | 非整数領域の格子点計数・整数凸包 | 追加整備済み | 上下境界の一次式にfloor-sumを適用する。整数凸包は各整数x列の最下端・最上端だけを凸包へ入れる。 ユーザー承認済みの有理凸領域。点集合を凸包に正規化し、境界込み。空・点・線分も扱う。計数結果BigInt。整数凸包だけは整数x列を走査し、予算超過をlength_errorで拒否する。 [実装](../computational_geometry/rational_lattice.hpp) / [説明・検証](computational_geometry/rational_lattice.md)。 | 追加 |
| <a id="h-11"></a>H-11 | 原点を共有する矩形の増分和集合面積 | 追加整備済み | x昇順・高さ降順のskylineをmapで保持し、新しい矩形に隠れた角を削除する。 [0,x]×[0,y]、x/yは非負int64。追加のみ、重複可。areaはBigInt。一般矩形の任意追加・削除を扱うAPIではない。 [実装](../computational_geometry/origin_rectangle_union.hpp) / [説明・検証](computational_geometry/origin_rectangle_union.md)。 | 追加 |
| <a id="h-12"></a>H-12 | 線分sweep・全交点列挙・Arrangement | 追加整備済み | x区間の開始・終了で候補を絞り、厳密な交点で全線分を分割して重複辺をまとめる。 点線分・端点接触・共線重複・多重点を含む。交差pairは点1個または重複区間の両端2個。Arrangementは幾何辺を統合し、元線分ID一覧を保持する。max_pairs=2000000、max_candidates=20000000で明示予算。 [実装](../computational_geometry/segment_arrangement.hpp) / [説明・検証](computational_geometry/segment_arrangement.md)。 | 追加 |
| <a id="h-13"></a>H-13 | 凸多角形intersectionの完成API | 追加整備済み | 片方の各辺の左側で他方を切断し、点・線分に退化した交差も厳密に残す。 入力の凸頂点集合をstrict CCWへ正規化する。戻り値は空・一点・線分・凸多角形を区別できる頂点列。整数・旧128bit有理点はrational::Pointへ明示変換できる。 [実装](../computational_geometry/convex_polygon_intersection.hpp) / [説明・検証](computational_geometry/convex_polygon_intersection.md)。 | 追加 |
| <a id="h-14"></a>H-14 | adaptive/exactな幾何predicates | 追加整備済み | 符号が0を跨ぐかを外向きの区間演算で確かめ、不確かな場合だけ全bitを使う。 整数はint64全域をBigIntへ拡張。浮動は実際に表現された有限radix-2 long doubleの値を判定する。外向きintervalで符号を確定できなければ厳密dyadic有理数へ切替。incircleはabcがCCWなら円内が正、CWなら逆。fast-math不可。 [実装](../computational_geometry/robust_predicates.hpp) / [説明・検証](computational_geometry/robust_predicates.md)。 | 追加 |
| <a id="h-15"></a>H-15 | 三角形内の点の個数クエリ | 追加整備済み | 三角形の各有向辺とKD-treeの矩形4隅を比較し、完全に内側なら個数をまとめて加える。 静的なint64点集合。重複点も個別に数える。CW可。退化三角形は境界込みなら線分／点上、strictなら0。 [実装](../computational_geometry/triangle_point_count.hpp) / [説明・検証](computational_geometry/triangle_point_count.md)。 | 追加 |
| <a id="h-16"></a>H-16 | 凸多角形の全頂点の最遠点 | 追加整備済み | 凸包上の最遠点添字の巡回単調性を使い、行最大値を分割統治で求める。 点集合は重複なし・strict凸位置。入力順は任意、凸包へ正規化し元添字へ戻す。空可、一点は自分。等距離のどれか1点を返し、全tie列挙はしない。距離はBigInt。 [実装](../computational_geometry/convex_farthest_neighbors.hpp) / [説明・検証](computational_geometry/convex_farthest_neighbors.md)。 | 追加 |
| <a id="h-17"></a>H-17 | 一般/凸多角形の和集合面積 | 追加整備済み | 全境界を分割し、各有界面の内点がどれかの領域に含まれる場合にだけ面積を足す。 各ringは単純、穴はouter内部で互いに内部が重ならないこと。ringの向きは任意。複数領域の重複・境界接触を統合。退化ringは面積0。自己交差ringの塗り規則は扱わない。 [実装](../computational_geometry/polygon_union_area.hpp) / [説明・検証](computational_geometry/polygon_union_area.md)。 | 追加, C3.22 |
| <a id="h-20"></a>H-20 | Delaunay Triangulation | 追加整備済み | 凸包の三角形分割へ点を挿入し、空円条件に違反する対角線を局所的に交換する。 任意精度有理座標。重複は統合し入力からの対応を返す。全共線は連続点間の辺、三角形なし。共円ではstrictなincircle違反だけをflipし、決定的な分割の一つを返す。人工的な巨大三角形を使わない。 [実装](../computational_geometry/delaunay.hpp) / [説明・検証](computational_geometry/delaunay.md)。 | C3.22 |
| <a id="h-21"></a>H-21 | Voronoi Diagram | 追加整備済み | Delaunayの隣接三角形の外心を結び、凸包辺には外向きのrayを付ける。cellは隣接siteとの垂直二等分線で切断する。 真の非有界辺をsegment/ray/lineで返す。origin+t·direction、tは[0,1]/[0,∞)/全実数。共円三角形間の長さ0の双対辺は省略。有限cellは明示矩形内で、sorted unique siteごと。重複対応はdelaunayのmap参照。 [実装](../computational_geometry/voronoi.hpp) / [説明・検証](computational_geometry/voronoi.md)。 | C3.22 |
| <a id="h-22"></a>H-22 | Dynamic Convex Hull | 追加整備済み | 点をブロックに分けて上下凸鎖を保持し、更新対象だけ再構築する。方向別の最大値は各鎖で二分探索する。 int64点のmultiset。eraseは1個削除、存在しなければfalse。support(direction)は内積最大の点をoptionalで返す。hullはstrict CCW・最小点始まり、更新後に参照無効。 [実装](../computational_geometry/dynamic_convex_hull.hpp) / [説明・検証](computational_geometry/dynamic_convex_hull.md)。 | C3.22 |
| <a id="h-23"></a>H-23 | Polygon Triangulation | 追加整備済み | 凸な耳の三角形内に他の頂点がないことを確かめて切り取り、両隣だけ耳候補を更新する。 穴なし単純ring。CW可、連続重複・閉点再掲・辺上の冗長点を除去し、元入力添字のCCW三角形を返す。空・全共線は空。自己交差・重なる辺は拒否。穴のbridge生成は含めない。 [実装](../computational_geometry/polygon_triangulation.hpp) / [説明・検証](computational_geometry/polygon_triangulation.md)。 | C3.22 |
| <a id="h-24"></a>H-24 | 三次元幾何 | 追加整備済み | 3Dの内積・外積で距離と交差を計算し、凸包は可視面を除いて境界と新点を結ぶ。 ユーザー承認済み範囲。Lineはorigin/direction、Planeはnormal·x=offset。交差は直線×平面、平面×平面、直線×球、平面×球、球×球。距離は点対、直線/線分/平面対点、直線対直線。凸包は厳密有理座標、dimension=-1/0/1/2/3、外向き三角形とvolume。浮動点の凸包も実際のdyadic値で厳密計算する。 [実装](../computational_geometry/geometry3d.hpp) / [説明・検証](computational_geometry/geometry3d.md)。 | C3.22 |

## 数値解析（追加カテゴリ）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="y-01"></a>Y-01 | 実数方程式の求根 | 追加整備済み | 安定な二次方程式・二分法・Newton・Brent。残差/区間幅・評価/反復予算と停止理由。 [API・使用例・検証](numerical_analysis/README.md)。 | 追加 |
| <a id="y-02"></a>Y-02 | 数値積分 | 追加整備済み | 等間隔Simpson・全体誤差優先の適応Simpson。深さ/評価予算と推定誤差の条件。 [API・使用例・検証](numerical_analysis/README.md)。 | 追加 |
| <a id="y-03"></a>Y-03 | 数値安定化ユーティリティ | 追加整備済み | Kahan・Neumaier補償和、Log-Sum-Exp、絶対＋相対誤差比較。幾何の判定とは別API。 [API・使用例・検証](numerical_analysis/README.md)。 | 追加 |
| <a id="y-04"></a>Y-04 | 多項式の浮動小数点実根列挙 | 追加整備済み | 二進係数を厳密有理数化したSturm分離。重根・近接根・予算終了時のcluster個数。 [API・使用例・検証](numerical_analysis/README.md)。 | 追加 |
| <a id="y-05"></a>Y-05 | 三重対角行列ソルバ | 追加整備済み | 隣接行pivot・第2上対角を保存するO(n)因子化。複数右辺、特異pivot閾値。 [API・使用例・検証](numerical_analysis/README.md)。 | 追加 |

## LP・マトロイド等（3.23）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="o-01"></a>O-01 | Simplex / 線形計画 | 追加整備済み | 二段階SimplexとBland規則。BigRationalの厳密解・双対/Farkas/ray証明、浮動型、符号自由変数、pivot予算。 [実装](../combinatorial_optimization/linear_program.hpp) / [API・使用例・検証](optimization_extensions/linear_program.md)。 | 追加 |
| <a id="o-02"></a>O-02 | 低次元線形計画 | 追加整備済み | 固定1〜3次元、符号自由の有理LP。記号的な箱の基底列挙で退化・実行不能・非有界を判定。候補数予算あり。 [実装](../combinatorial_optimization/low_dimensional_lp.hpp) / [API・使用例・検証](optimization_extensions/low_dimensional_lp.md)。 | 追加 |
| <a id="o-03"></a>O-03 | Matroid Union / Matroid Partition | 追加整備済み | 独立性oracle間の交換路で最大部分集合を分配し、所属・集合・全体分割の成否を復元。 [実装](../combinatorial_optimization/matroid_union.hpp) / [API・使用例・検証](optimization_extensions/matroid_union.md)。 | 追加 |
| <a id="o-04"></a>O-04 | 分数計画 | 追加整備済み | 大域最大化oracleを使うDinkelbachと比率二分。全域で正の分母下限、許容誤差・候補・上下界・停止状態。 [実装](../combinatorial_optimization/fractional_programming.hpp) / [API・使用例・検証](optimization_extensions/fractional_programming.md)。 | 追加 |
| <a id="o-20"></a>O-20 | Weighted Matroid Intersection | 追加整備済み | 重みと辺数で最短路を選ぶ交換グラフ。負重み・要素数ごとの最適集合・全体の最大重み。既存の独立性oracleを共用。 [実装](../combinatorial_optimization/weighted_matroid_intersection.hpp) / [API・使用例・検証](optimization_extensions/weighted_matroid_intersection.md)。 | C3.23 |
| <a id="o-21"></a>O-21 | Base Polyhedron | 追加整備済み | 一般劣モジュラoracleの貪欲基、基多面体上の線形最大/最小化、Lovász拡張。空集合のoffsetも扱う。 [実装](../combinatorial_optimization/base_polyhedron.hpp) / [API・使用例・検証](optimization_extensions/base_polyhedron.md)。 | C3.23 |
| <a id="o-22"></a>O-22 | Submodular Optimization | 追加整備済み | 厳密有理LPと貪欲基によるcutting-plane最小化。予算時も候補値と下界を保持。単調・個数制限の近似最大化も公開。最小化の最悪計算量は指数。 [実装](../combinatorial_optimization/submodular_optimization.hpp) / [API・使用例・検証](optimization_extensions/submodular_optimization.md)。 | C3.23 |
| <a id="o-23"></a>O-23 | 特殊な整数計画法 | 追加整備済み | ユーザー承認済みの有限整数上下限付き線形計画。厳密LP緩和・分枝限定、解・最適性・ノード/LP予算・目的上界。 [実装](../combinatorial_optimization/integer_program.hpp) / [API・使用例・検証](optimization_extensions/integer_program.md)。 | C3.23 |

## 高度なゲーム理論（3.24）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="b-20"></a>B-20 | 超限的なゲーム値・高度な組合せゲーム理論 | 追加整備済み | ユーザー承認済みの有理関数Q(ω)。無限大ω・無限小1/ω、四則演算・比較・標準部分・数としての勝敗。多項式GCDはSturm実根分離と共有。 [実装](../game_theory/omega_rational.hpp) / [API・使用例・検証](optimization_extensions/omega_rational.md)。 | C3.24 |

## 探索・ヒューリスティック（3.25）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="e-01"></a>E-01 | 汎用Undo Manager・score検算 | 追加整備済み | 変更前の値や逆操作を記録し、履歴の末尾から逆順に戻す。スコア差分は全再計算と照合する。 [API・条件・検証](search_parsing_extensions/undo_manager.md)。 | 追加 |
| <a id="e-02"></a>E-02 | IndexSet | 追加整備済み | 密な配列と逆位置を持ち、削除位置へ末尾の要素を移して穴を埋める。 [API・条件・検証](search_parsing_extensions/index_set.md)。 | 追加 |
| <a id="e-03"></a>E-03 | TSP近傍 | 追加整備済み | 取り除く辺と加える辺だけで差分を求め、列の反転・回転で巡回路を変更する。 [API・条件・検証](search_parsing_extensions/tsp_neighborhood.md)。 | 追加 |
| <a id="e-04"></a>E-04 | SA受理判定の高速化 | 追加整備済み | 均等な確率区間の中点について−log(u)を前計算し、loss<T×閾値で採否を決める。 [API・条件・検証](search_parsing_extensions/fast_annealing_acceptance.md)。 [確率誤差・性能実測](search_parsing_extensions/benchmark.md)。 | 追加 |
| <a id="e-05"></a>E-05 | 差分更新型Beam Search | 追加整備済み | 候補木の共通prefixをDFSし、辺をapply/undoしながら状態を共有する。残す候補だけを次のfrontierへ進める。 [API・条件・検証](search_parsing_extensions/delta_beam_search.md)。 | 追加 |
| <a id="e-06"></a>E-06 | Chokudai Search | 追加整備済み | 深さ別queueを消さずに保持し、後の周回でも見送った候補から探索を続ける。 [API・条件・検証](search_parsing_extensions/chokudai_search.md)。 | 追加 |
| <a id="e-20"></a>E-20 | 多点並列探索 | 追加整備済み | 開始点ごとに乱数系列を固定して並列に探索し、終了後にrun番号順で最良解を選ぶ。 [API・条件・検証](search_parsing_extensions/parallel_search.md)。 | C3.25 |
| <a id="e-21"></a>E-21 | 特殊な確率分布の高速サンプリング | 追加整備済み | Alias列を大小の重みで埋める。非復元抽出は指数時計、連続分布は逆関数とBox–Muller変換を使う。 [API・条件・検証](search_parsing_extensions/distribution_sampling.md)。 ユーザー承認済みのAlias・非復元・正規/指数/切断指数。 | C3.25 |

## 構文解析・オートマトン（新カテゴリ候補）

| ID | 項目 | 状態 | 残る機能・既存実装との差 | 出所 |
|---|---|---|---|---|
| <a id="x-01"></a>X-01 | 再帰下降parser・構文木のライブラリ化 | 追加整備済み | 優先順位に従って再帰下降し、子を先に格納した構文木を反復的に評価する。 [API・条件・検証](search_parsing_extensions/expression_parser.md)。 | 追加 |
| <a id="x-02"></a>X-02 | DFA/NFAの共通操作 | 追加整備済み | ε閉包の集合をDFA状態にし、受理と遷移先の同値類で分割を細かくして最小化する。 [API・条件・検証](search_parsing_extensions/automata.md)。 | 追加 |
| <a id="x-03"></a>X-03 | 正規表現→NFA / Thompson構成 | 追加整備済み | 連接・選択・繰返しをε辺で結び、入力を読むごとに到達可能な状態集合を進める。 [API・条件・検証](search_parsing_extensions/regex_nfa.md)。 | 追加 |

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

ファイル名だけで未実装と判断せず、公開メソッドと共有先を確認しました。とくにMerge Sort TreeはFC版、動的列はTreap版、一般静的MEXはgrundy_mexで既に満たします。初回監査では旧コードがあるFastIO・円接線・parserを整備待ちとしていましたが、現在は該当項目も整備済みです。

照合時はrg --filesと機能名/公開メソッドの検索を併用し、該当ヘッダ・個別説明・既存検証コードを確認しました。初回監査は文書の追加だったため全アルゴリズムの再テストはせず、ローカル参照先・ID重複・元Cの全件対応・件数を検査しています。前回のB実装の通常/Sanitizer/回帰結果は引継書に記録済みです。

今後は各IDの残る仕様を満たした時点で状態と根拠を更新し、元Cの対応表も同時に維持してください。任意の追加方式や高速化を、既存機能全体が未実装であるという意味にはしません。

2026-10-19のF-01〜F-07追加分は `python3 verify/run_foundation.py` / `--sanitize`、図解モデル・ブラウザ検証を使用します。[検証範囲](basic_utility/foundation.md)。

2026-10-19のF-08〜F-10追加分は `python3 verify/run_foundation_structures.py` / `--sanitize`、5モードの図解モデル・ブラウザ検証を使用します。[検証範囲](foundation_structures/README.md)。

2026-10-19のF-11追加分は `python3 verify/run_big_integer.py` / `--sanitize`、4モードの図解モデル・ブラウザ検証を使用します。[検証範囲・性能](basic_utility/big_integer.md)。

2026-10-19のF-12〜F-17追加分は `python3 verify/run_foundation_tools.py` / `--sanitize`、6モードの図解モデル・ブラウザ検証を使用します。[検証範囲](foundation_tools/README.md)。

2026-10-20のU-01〜U-11・U-20〜U-22は `make verify-utility-search` と14モードの図解検証を使用します。[検証範囲](utility_search/README.md)。

2026-10-20のD-01は `make verify-static-query` と同カテゴリの6モードの図解検証を使用します。[対応範囲・検証](static_query/swag_deque.md)。

2026-10-24のD-02〜D-10は `make verify-data-structure-extensions` と9モードの図解を使用します。[対応範囲・検証](data_structure_extensions/README.md)。D-11〜D-19の登録はありません。
