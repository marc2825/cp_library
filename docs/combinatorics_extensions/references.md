# C追加項目の一次資料・設計判断

2026-10-26確認。コードは本リポジトリ向けに実装し、既存のLucas、NTT/FPS、BigInt、SPF、mod演算を共有しています。外部ライブラリのソースを移植していません。

| 対象 | 一次資料 | 今回の設計 |
|---|---|---|
| q二項係数 | [NIST DLMF 26.16](https://dlmf.nist.gov/26.16)（q多項係数） | 有限体のq-Lucasで消失する因子を分離し、小素数全表の予算を明示 |
| 境界付き列 | [Library Checker問題](https://judge.yosupo.jp/problem/number_of_increasing_sequences_between_two_sequences)、[Krattenthalerの格子路論文](https://www.mat.univie.ac.at/~kratt/artikel/count1.pdf) | 格子路への変換を基に、長方形の4境界転送カーネルを独立導出。一般の非単調入力を正規化し、座標幅の上限を明示 |
| Stirling | [NIST DLMF 26.8](https://dlmf.nist.gov/26.8)（定義・漸化式） | 標数pの上昇階乗と完全対称式の周期からuint64問い合わせ式を導出。p=2も同じ契約 |
| ラベルなし木 | [Pippengerの木列挙論文（通常木の母関数・Otterの関係を含む）](https://www.cs.ubc.ca/sites/default/files/tr/2000/TR-2000-04_0.pdf) | 根付きEuler積と根を外す公式を任意精度で評価。空木は0 |
| RSK / Young / Hook | [PakのHook Length Formula論文](https://www.math.ucla.edu/~pak/papers/hl7.pdf)、[Stanfordの組合せ論講義ノート](https://web.stanford.edu/~lindrew/18.212.pdf) | 行挿入の等値規則、逆の右端優先、callbackの所有権、合成数法で除算しない実装を定義 |
| Lah / Euler | [NIST DLMF 26.8](https://dlmf.nist.gov/26.8)、[NIST DLMF 26.9](https://dlmf.nist.gov/26.9) | Lahの階乗分離とEuler積の対数微分を畳み込み/FPSへ接続 |

数学的定義や公式の参照と、公開APIの資源上限・例外・入力規約を区別しています。C-02の長方形転送式とC-03の小素数式の導出は各説明に記載しました。オンラインジャッジへの提出はしていません。
