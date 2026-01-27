## Getting Started

Welcome to the VS Code Java world. Here is a guideline to help you get started to write Java code in Visual Studio Code.

## Folder Structure

The workspace contains two folders by default, where:

- `src`: the folder to maintain sources
- `lib`: the folder to maintain dependencies

Meanwhile, the compiled output files will be generated in the `bin` folder by default.

> If you want to customize the folder structure, open `.vscode/settings.json` and update the related settings there.

## Dependency Management

The `JAVA PROJECTS` view allows you to manage your dependencies. More details can be found [here](https://github.com/microsoft/vscode-java-dependency#manage-dependencies).

## Run the simulator (PowerShell)

### Prerequisites

- Install a JDK locally (recommended: Temurin / Microsoft Build of OpenJDK).
- This repository does **not** include OpenJDK binaries (they are ignored via `.gitignore`).

You can either:

- Ensure `java`/`javac` are available on your `PATH`, or
- Set `JAVA_HOME` to your JDK installation directory.

Build (clean compile to `out/`):

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

Run:

```powershell
& $java -cp ".\out;lib\*" sim.Main
```

Controls:

- `Space`: start/stop
- `R`: reset positions and ball
