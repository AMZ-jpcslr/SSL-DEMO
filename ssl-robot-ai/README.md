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

This repo includes a bundled JDK under `openJdk-25/`.

Build (clean compile to `out/`):

```powershell
cd "c:\Users\yomas\Desktop\SSL-AI\ssl-robot-ai"
if (Test-Path .\out) { Remove-Item -Recurse -Force .\out }
New-Item -ItemType Directory -Path .\out | Out-Null

$sources = Get-ChildItem -Path .\src\main\java -Recurse -Filter *.java | ForEach-Object { $_.FullName }
& .\openJdk-25\bin\javac.exe -encoding UTF-8 -d .\out -cp "lib\*" $sources
```

Run:

```powershell
& .\openJdk-25\bin\java.exe -cp ".\out;lib\*" sim.Main
```

Controls:

- `Space`: start/stop
- `R`: reset positions and ball
