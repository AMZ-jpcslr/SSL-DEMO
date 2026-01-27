## SSL Robot AI シミュレーター

ロボカップSSL風の2Dシミュレーター（Java）です。`sim.Main` を起動するとGUIが立ち上がり、ロボットが自律的に動きます。

### ざっくり図（イメージ）

```
	+---------------------------------------+
	|  BLUE o o o                o o o RED  |
	|                                       |
	|                 (  ball  )            |
	|                                       |
	+---------------------------------------+
		 Space: start/stop     R: reset
```

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
