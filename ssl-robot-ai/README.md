## SSL Robot AI シミュレーター

SSL風の2Dシミュレーター（Java）です。`sim.Main` を起動するとGUIが立ち上がり、ロボットが自律的に動きます。

## フォルダ構成

- `src/` : Javaソース
- `lib/` : 依存ライブラリ（クラスパスに追加）
- `out/` : ローカルビルド成果物（手順で作成）

## 実行手順（Windows / PowerShell）

### 事前準備（JDKは各自用意）

- JDKをローカルにインストールしてください（例: Temurin / Microsoft Build of OpenJDK）。
- このリポジトリにはOpenJDK本体は含めません（容量が大きいため `.gitignore` で除外しています）。

どちらかを満たせばOKです:

- `java` / `javac` が `PATH` から実行できる
- `JAVA_HOME` にJDKのパスを設定している

### ビルド（`out/` にクリーンコンパイル）

```powershell
cd "ssl-robot-ai"

# Optional: point this to your installed JDK.
# $env:JAVA_HOME = "C:\Program Files\Eclipse Adoptium\jdk-21.0.2.13-hotspot"

$javac = if ($env:JAVA_HOME) { Join-Path $env:JAVA_HOME "bin\javac.exe" } else { "javac" }
$java  = if ($env:JAVA_HOME) { Join-Path $env:JAVA_HOME "bin\java.exe" } else { "java" }

if (Test-Path .\out) { Remove-Item -Recurse -Force .\out }
New-Item -ItemType Directory -Path .\out | Out-Null

$sources = Get-ChildItem -Path .\src\main\java -Recurse -Filter *.java | ForEach-Object { $_.FullName }
& $javac -encoding UTF-8 -d .\out -cp "lib\*" $sources
```

### 実行

```powershell
& $java -cp ".\out;lib\*" sim.Main
```

## 操作

- `Space`: start/stop
- `R`: reset positions and ball

---

## スコアリング/学習（コードの場所と変数メモ）

ここでは「フィールド上の候補位置をどう採点しているか（スコアリング）」と「学習（オンライン更新）をどこで使っているか」を、該当ファイルと主要変数名ベースでまとめてます。

### フィールドのスコアリング（位置評価）

- `tactics.PositionScorer`（`src/main/java/tactics/PositionScorer.java`）
	- `score(WorldState world, Robot self, double x, double y, int teamSign)` が「位置 (x,y) の良さ」を返す関数インターフェースです。
	- `teamSign` は攻撃方向の符号（+1/-1）として使われます。

- `tactics.ScoreGrid`（`src/main/java/tactics/ScoreGrid.java`）
	- `findBest(..., double step, PositionScorer scorer)` でフィールドを格子状にサンプリングして最大スコア地点を探します。
	- 主要変数:
		- `step`: グリッド間隔（m）
		- `margin`: 壁際に寄りすぎないための余白（ロボット半径など）
		- `bestX/bestY/best`: 現在の最良地点とスコア

- `tactics.TacticalScorers`（`src/main/java/tactics/TacticalScorers.java`）
	- 実際の「採点式（ヒューリスティック）」がここにあります。
	- 例: `attackOffBall()` は “オフボールの攻撃位置” を評価します。
	- よく出てくる変数（攻撃の例）:
		- `open2`: 近くに敵がいない（オープン）ほど加点
		- `mateMin` / `mate1`: 味方と近すぎないように加点/減点
		- `passOptions` / `passPts`: 味方→候補点へのパスラインが通る本数で加点
		- `interceptable` / `interceptPenalty`: パスが奪われやすいなら減点
		- `shootBlocked` / `shoot2`: ゴール方向シュートが通るなら加点
		- `nearBallPenalty`: ボールに近すぎる位置を減点

### 学習（オンライン学習）の採用箇所

このシミュレーターは「外部MLライブラリなし」で、軽量なオンライン学習（重みの逐次更新）を使っています。

- `sim.Main`（`src/main/java/sim/Main.java`）
	- 試行（パス/シュート/ポジション）の特徴量を保存し、成功/失敗に応じて `applyReward(...)` を呼んで学習させています。
	- 主要変数（学習用の一時保持）:
		- `LAST_PASS_FEATURES`: 直近のパス評価に使った特徴量
		- `LAST_ACTION_FEATURES` / `LAST_ACTION_SHOOT`: 直近の「シュートorパス」判断の特徴量と行動
		- `LAST_ATTACK_POS_FEATURES[]` / `LAST_DEF_POS_FEATURES[]`: 各ロボの直近ポジション特徴量（攻撃/守備）

- `ai.PassLearning`（`src/main/java/ai/PassLearning.java`）
	- 「誰にパスするか」を線形モデル `score = W·features` で評価し、報酬で `W[]` を更新します。
	- 主要変数:
		- `LR` / `L2`: 学習率とL2正則化（暴れ防止）
		- `W[]`: 重み（`pass-weights.properties` に保存）
		- 特徴量インデックス: `F_FORWARD`, `F_OPENNESS`, `F_LANE`, `F_RANGE`, `F_CENTRAL`
			- 意味: 前進量 / 受け手のオープンさ / パスレーンの安全 / 距離の好み / 中央寄り

- `ai.ActionLearning`（`src/main/java/ai/ActionLearning.java`）
	- 「シュートするかパスするか」を `P(shoot)=sigmoid(W·features)` で学習します。
	- 主要変数:
		- `chooseShoot(features, epsilon)`: `epsilon` は探索（ランダム行動）の割合
		- `W[]`: 重み（`action-weights.properties` に保存）
		- 特徴量: `F_IN_SHOOT_ZONE`, `F_GOAL_LANE`, `F_DIST_TO_GOAL`, `F_BALL_X_ATTACK`, `F_BEST_PASS_SCORE`, `F_SAFE_PASS_COUNT`

- `tactics.PositionLearning`（`src/main/java/tactics/PositionLearning.java`）
	- オフボール位置に「学習ボーナス」を加える仕組みです（ヒューリスティック + 学習の足し算）。
	- 主要変数:
		- `WA[]` / `WD[]`: 攻撃/守備の重み（`position-weights.properties` に保存）
		- `LR` / `L2`: 学習率とL2正則化
		- 攻撃特徴量: `A_FORWARD`(前進), `A_OPEN`(オープン), `A_LANE`(レーン), `A_RANGE`(距離), `A_CENTRAL`(中央), `A_TEAMSPACE`(味方との距離)
		- 守備特徴量: `D_GOALSIDE`(ゴール側), `D_LINEHOLD`(ライン維持), `D_LANECUT`(レーン遮断), `D_MARKDIST`(マーク距離), `D_MOVE`(移動コスト)

### 学習データ（重みファイル）

学習結果は実行ディレクトリ（`ssl-robot-ai/`）に次のファイルとして保存されます。

- `pass-weights.properties`
- `action-weights.properties`
- `position-weights.properties`

リセットしたい場合は、これら3ファイルを削除するとデフォルト重みから再スタートします。
