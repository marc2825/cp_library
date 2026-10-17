# 一次資料と設計判断

2026-10-17に以下の著者資料・実装・公式問題仕様を確認しました。外部コード・説明文は転載せず、定義と既存APIに合わせて実装しています。

| 一次資料 | 確認した内容・今回の設計 |
|---|---|
| [CP-Algorithms: Suffix Automaton](https://cp-algorithms.com/string/suffix-automaton.html) | endpos、clone、出現回数、LCS。今回は任意byteの疎なmap遷移と、追加後に無効化する出現数キャッシュを公開 |
| [CP-Algorithms: Lyndon Factorization](https://cp-algorithms.com/string/lyndon_factorization.html) | Duval法と非増加順の因子。今回はunsigned byte順の半開区間を返し、空と重複因子を保持 |
| [Nyaan: 2D Rolling Hash](https://nyaannyaan.github.io/library/string/rolling-hash-2d.hpp.html) | 二方向の冪と矩形の包除。今回は既存1D版と同じ二つの素数法・byte+1、固定/指定base、空と範囲検査を採用 |
| [Nyaan: Nim Product](https://nyaannyaan.github.io/library/math/nimber.hpp.html) / [hitonanode: Nimber](https://hitonanode.github.io/cplib-cpp/number/nimber.hpp.html) | 64bit nim値の体、Fermatの2冪による二次拡大。今回は小表をmexで構築し、SIMD・128bit演算に依存しない |
| [CP-Algorithms: Sprague–Grundy](https://cp-algorithms.com/game_theory/sprague-grundy-nim.html) / [IIT Bombay: Combinatorial Game Theory講義](https://www.ieor.iitb.ac.in/sites/default/files/LectureNotesCombGameIE619_Spring2025_0.pdf) | Grundy列の観測と周期性。今回は任意列の候補と有限引き算ゲームの窓一致による証明を別APIにする |
| [Zobrist: Technical Report 88](https://research.cs.wisc.edu/techreports/1970/TR88.pdf) | ゲームの状態に乱数を割り当ててXORする方法。今回は位置×値の固定表を既存SplitMix64から作り、old/newの差分を公開 |
| [Glover: Tabu Search: A Tutorial](https://pubsonline.informs.org/doi/10.1287/inte.20.4.74) | 禁止条件・aspiration・悪化手。今回は訪問keyと一定tenureの最小構成で、期限と同点処理を明示 |
| [Library Checker: Lyndon公式仕様](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/string/lyndon_factorization/task.md) | 因子の境界列0=a0<...<ak=Nを出力 |
| [Library Checker: LCS公式仕様](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/string/longest_common_substring/task.md) | 両方の半開区間を返す。今回は同長時の決定規則も契約に含める |
| [Library Checker: Nim Product公式仕様](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/number_theory/nim_product_64/task.md) | uint64全域、複数入力、mexによる定義 |

Beamの重複除去は既存の安定なscore順を保ち、keyごとの最良代表の後に幅を制限する設計です。異なるkeyのhash衝突はequalで区別します。差分SAは既存の[温度・乱数・採否](../heuristic_search/simulated_annealing.md)を共有し、候補をin-placeで作り、棄却時とaccept例外時にundoします。

観測した有限列だけでは一般のGrundy列の未来を証明できません。有限引き算ゲームは、最大手Mの直前M値が将来を決めることを利用します。窓の一致長がM未満なら、見かけの反復が何回あっても外挿しません。証明は固定された正の手集合と通常ルールに限ります。

CP-Algorithmsの文章はCC BY-SA 4.0です。今回コード・原文の転載はしていません。SplitMix64は既存Randomの出典とpublic-domain表記を保持します。オンラインACは未確認です。
