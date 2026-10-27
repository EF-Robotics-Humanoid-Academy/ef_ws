<#
.SYNOPSIS
Teleop Studio launcher (Windows) - dev and packaged modes, auto-locate.

.DESCRIPTION
Modes:
  Packaged: env\ (conda-pack) + gmr\ (repo) next to this script.
  Dev: no env\, use conda run to launch in the target env; repo is parent dir.

Uses `conda run -n <env>` to avoid execution-policy issues with conda-hook.ps1
and environment-variable isolation between cmd bat and PowerShell.

Override:
  $env:CONDA_ROOT  - conda install dir (auto-detected from PATH by default)
  $env:CONDA_ENV   - conda env name (default: try "gmr", fall back to "base")
#>
$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# --- 1) Locate package root (search upward for env\ + gmr\ pair) ---
$PkgRoot = $ScriptDir
while ($PkgRoot) {
    if ((Test-Path (Join-Path $PkgRoot "env")) -and (Test-Path (Join-Path $PkgRoot "gmr"))) {
        break
    }
    $Parent = Split-Path $PkgRoot -Parent
    if (-not $Parent -or $Parent -eq $PkgRoot) {
        $PkgRoot = $null
        break
    }
    $PkgRoot = $Parent
}

# --- 2) Locate Python environment ---
$EnvActivate = if ($PkgRoot) { Join-Path $PkgRoot "env\Scripts\activate.bat" } else { $null }

if ($EnvActivate -and (Test-Path $EnvActivate)) {
    # Packaged mode: use the bundled conda-pack environment.
    # 不依赖 activate.bat（它是 cmd 批处理，PowerShell 调用时 PATH 不回传），
    # 直接在 PowerShell 中手动设置 PATH/CONDA_PREFIX/PYTHONPATH，用 env\python.exe 绝对路径运行。
    # Windows 上不需要 conda-unpack（无 shebang，activate.bat 用 %~dp0 相对路径，无硬编码前缀）
    $EnvDir = Join-Path $PkgRoot "env"
    $EnvPython = Join-Path $EnvDir "python.exe"

    # --- 3) Locate GMR repo ---
    $GmrRoot = if (Test-Path (Join-Path $PkgRoot "gmr")) {
        Join-Path $PkgRoot "gmr"
    } else {
        Join-Path $ScriptDir ".."
    }

    # 手动设置 conda 环境变量（模拟 activate.bat 的效果，但在 PowerShell 进程内生效）
    $env:CONDA_PREFIX = $EnvDir
    $env:PYTHONPATH = "$GmrRoot;$env:PYTHONPATH"
    # PATH 前置 env 目录 + 子目录（和 activate.bat 一致）
    $envPathEntries = @(
        $EnvDir,
        (Join-Path $EnvDir "Library\mingw-w64\bin"),
        (Join-Path $EnvDir "Library\usr\bin"),
        (Join-Path $EnvDir "Library\bin"),
        (Join-Path $EnvDir "Scripts")
    )
    $env:PATH = ($envPathEntries -join ";") + ";$env:PATH"

    # 重写 editable 包的源码路径（在当前 PowerShell 进程中直接执行，不依赖 .bat/.ps1 钩子）
    # editable finder（__editable___*_finder.py）的 MAPPING 中硬编码了打包机的路径，
    # 需要重写为本地的 gmr\ 目录路径，否则导入 general_motion_retargeting / sonic_sim 会失败。
    $packRootFile = Join-Path $EnvDir ".pack_source_root"
    $sitePkgs = Join-Path $EnvDir "Lib\site-packages"
    if ((Test-Path $packRootFile) -and (Test-Path $sitePkgs)) {
        $oldPath = (Get-Content $packRootFile -Raw).Trim()
        if ($oldPath -and $GmrRoot -ne $oldPath) {
            # 1) Rewrite __editable___*_finder.py MAPPING paths
            Get-ChildItem (Join-Path $sitePkgs "__editable___*_finder.py") -ErrorAction SilentlyContinue | ForEach-Object {
                $c = Get-Content $_.FullName -Raw
                $c = $c.Replace($oldPath.Replace('\', '\\'), $GmrRoot.Replace('\', '\\'))
                $c = $c.Replace($oldPath.Replace('\', '/'), $GmrRoot.Replace('\', '/'))
                [System.IO.File]::WriteAllText($_.FullName, $c, [System.Text.UTF8Encoding]::new($false))
            }
            # 2) Rewrite direct_url.json (JSON 中路径用 \\ 转义)
            Get-ChildItem (Join-Path $sitePkgs "*.dist-info\direct_url.json") -ErrorAction SilentlyContinue | ForEach-Object {
                $data = Get-Content $_.FullName -Raw | ConvertFrom-Json
                $oldUri = ([System.Uri]$oldPath).AbsoluteUri.TrimEnd('/')
                $newUri = ([System.Uri]$GmrRoot).AbsoluteUri.TrimEnd('/')
                if ($data.url -eq $oldUri -or $data.url.StartsWith($oldUri + '/')) {
                    $data.url = $data.url.Replace($oldUri, $newUri)
                    [System.IO.File]::WriteAllText($_.FullName, ($data | ConvertTo-Json -Depth 10), [System.Text.UTF8Encoding]::new($false))
                }
            }
            # 3) Remove legacy egg-link files
            Get-ChildItem (Join-Path $sitePkgs "*.egg-link") -ErrorAction SilentlyContinue | Remove-Item -Force
            # 更新 .pack_source_root，避免下次重复重写
            Set-Content -Path $packRootFile -Value $GmrRoot
        }
    }

    # 直接用 env\python.exe 绝对路径启动（不依赖 PATH）
    Push-Location $GmrRoot
    & $EnvPython -m teleop_studio.app @args
    Pop-Location
} else {
    # Dev mode: auto-detect conda, use `conda run` to launch in the target env.
    # All conda invocations use the absolute path to conda.exe so the script
    # works even when `conda` is not on PATH (no `conda init` in this shell).

    # 如果 CONDA_PREFIX 指向一个 conda-pack 打包环境（非 conda 管理的环境），
    # conda run 会报错。清除残留的 conda-pack 环境变量。
    if ($env:CONDA_PREFIX -and -not (Test-Path (Join-Path $env:CONDA_PREFIX "conda-meta\history"))) {
        Remove-Item Env:\CONDA_PREFIX -ErrorAction SilentlyContinue
    }

    # --- 2a) Resolve conda.exe absolute path ---
    $CondaExe = $null

    if ($env:CONDA_ROOT) {
        # User provided CONDA_ROOT explicitly
        $candidate = Join-Path $env:CONDA_ROOT "Scripts\conda.exe"
        if (Test-Path $candidate) { $CondaExe = $candidate }
    }

    if (-not $CondaExe) {
        # Probe 1: conda as a real .exe on PATH
        $condaCmd = Get-Command conda -ErrorAction SilentlyContinue
        if ($condaCmd) {
            if ($condaCmd.CommandType -eq 'Application' -and $condaCmd.Source) {
                $CondaExe = $condaCmd.Source
            } elseif ($condaCmd.CommandType -eq 'Function') {
                # `conda init powershell` wraps conda in a function; the real
                # executable lives in <CONDA_ROOT>\Scripts\conda.exe. Extract
                # it from the function body's path references.
                $m = [regex]::Match($condaCmd.Definition, '(?i)([A-Za-z]:\\[^\s"'']+\\Scripts\\conda\.exe)')
                if ($m.Success) { $CondaExe = $m.Groups[1].Value }
            }
        }
    }

    if (-not $CondaExe) {
        # Probe 2: registry (Uninstall keys point at conda install dirs)
        $regKeys = @(
            'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
            'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*',
            'HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*'
        )
        foreach ($key in $regKeys) {
            Get-ItemProperty $key -ErrorAction SilentlyContinue | ForEach-Object {
                if ($_.DisplayName -match 'Anaconda|Miniconda') {
                    $loc = $_.InstallLocation
                    if ($loc) {
                        $c = Join-Path $loc "Scripts\conda.exe"
                        if (Test-Path $c) { $CondaExe = $c }
                    }
                }
            }
            if ($CondaExe) { break }
        }
    }

    if (-not $CondaExe) {
        # Probe 3: common install locations (per-user + both ProgramData drives)
        $candidates = @(
            (Join-Path $env:USERPROFILE "miniconda3"),
            (Join-Path $env:USERPROFILE "anaconda3"),
            (Join-Path $env:USERPROFILE "AppData\Local\miniconda3"),
            (Join-Path $env:USERPROFILE "AppData\Local\anaconda3"),
            "C:\ProgramData\miniconda3",
            "C:\ProgramData\anaconda3",
            "D:\ProgramData\miniconda3",
            "D:\ProgramData\anaconda3"
        )
        $found = $candidates | Where-Object { Test-Path (Join-Path $_ "Scripts\conda.exe") } | Select-Object -First 1
        if ($found) { $CondaExe = Join-Path $found "Scripts\conda.exe" }
    }

    if (-not $CondaExe -or -not (Test-Path $CondaExe)) {
        Write-Error "conda not found. Set `$env:CONDA_ROOT to your conda install dir."
        Read-Host "Press Enter to exit"
        exit 1
    }

    # Derive CONDA_ROOT from conda.exe path (Scripts\conda.exe -> up two levels)
    if (-not $env:CONDA_ROOT) {
        $env:CONDA_ROOT = Split-Path (Split-Path $CondaExe -Parent) -Parent
    }
    Write-Host "[teleop_studio] conda: $CondaExe (CONDA_ROOT=$env:CONDA_ROOT)" -ForegroundColor DarkGray

    # --- 2b) Determine env name: explicit > gmr (if exists) > base ---
    $EnvName = if ($env:CONDA_ENV) {
        $env:CONDA_ENV
    } else {
        # List envs via absolute conda.exe (no dependency on PATH)
        $envsJson = & $CondaExe env list --json 2>$null
        $envs = $envsJson | ConvertFrom-Json
        if ($envs.envs -contains (Join-Path $env:CONDA_ROOT "envs\gmr")) {
            "gmr"
        } elseif ($envs.envs -contains $env:CONDA_ROOT) {
            "base"
        } else {
            "base"
        }
    }
    Write-Host "[teleop_studio] env: $EnvName" -ForegroundColor DarkGray

    # --- 3) Locate GMR repo ---
    $GmrRoot = if ($PkgRoot -and (Test-Path (Join-Path $PkgRoot "gmr"))) {
        Join-Path $PkgRoot "gmr"
    } else {
        Join-Path $ScriptDir ".."
    }

    # 直接用 gmr 环境的 python.exe 绝对路径运行（不依赖 conda run，避免 conda hook 干扰）
    $GmrPython = Join-Path $env:CONDA_ROOT "envs\$EnvName\python.exe"
    if (-not (Test-Path $GmrPython)) {
        # 回退：用 conda run
        $env:PYTHONPATH = "$GmrRoot;$env:PYTHONPATH"
        Push-Location $GmrRoot
        & $CondaExe run --no-capture-output -n $EnvName python -m teleop_studio.app @args
        Pop-Location
    } else {
        # 设置 PYTHONPATH，直接运行
        $env:PYTHONPATH = "$GmrRoot;$env:PYTHONPATH"
        Push-Location $GmrRoot
        & $GmrPython -m teleop_studio.app @args
        Pop-Location
    }
}
