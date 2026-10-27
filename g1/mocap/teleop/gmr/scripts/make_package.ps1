# 打包 Teleop Studio 为自包含、可重定位安装包（Windows）。
# 须在已就绪 gmr conda 环境的 Windows 上运行（同一平台只能打该平台的包）。
#
# 产物结构：
#   dist\TeleopStudio-windows\
#     env\                   conda-pack 打包的 gmr 环境（启动器自动设置环境变量）
#     gmr\                   GMR 仓库源码
#     run_teleop_studio.ps1  相对自身定位 env\ 与 gmr\ 的启动器
#   dist\TeleopStudio-windows-<ver>.zip  可分发压缩包
#
# 安装：用户解压到任意目录，双击 run_teleop_studio.ps1 即可，路径全自动确定。
param([string]$Version = (Get-Date -Format "yyyyMMdd"))
$ErrorActionPreference = "Stop"

$GmrRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$Stage   = Join-Path $GmrRoot "dist\TeleopStudio-windows"
$OutName = "TeleopStudio-windows-$Version"

Write-Host "[1/7] 检查 conda-pack"
# conda-pack 不能装到 base（与 conda 自带的 setuptools 冲突，报 RemoveError），
# 装到 gmr 环境里。先解析 conda.exe 绝对路径，不依赖 PATH。
$_condaExe = $null
if (Get-Command conda -ErrorAction SilentlyContinue) {
    $_cmd = Get-Command conda
    if ($_cmd.CommandType -eq 'Application' -and $_cmd.Source) {
        $_condaExe = $_cmd.Source
    } elseif ($_cmd.CommandType -eq 'Function') {
        $_m = [regex]::Match($_cmd.Definition, '(?i)([A-Za-z]:\\[^\s"'']+\\Scripts\\conda\.exe)')
        if ($_m.Success) { $_condaExe = $_m.Groups[1].Value }
    }
}
if (-not $_condaExe) {
    foreach ($_d in @("$env:USERPROFILE\miniconda3", "$env:USERPROFILE\anaconda3",
                      "C:\ProgramData\miniconda3", "C:\ProgramData\anaconda3",
                      "D:\ProgramData\miniconda3", "D:\ProgramData\anaconda3")) {
        $_c = Join-Path $_d "Scripts\conda.exe"
        if (Test-Path $_c) { $_condaExe = $_c; break }
    }
}
if (-not $_condaExe) { throw "conda.exe 未找到，请设置 PATH 或 CONDA_ROOT" }
# 从 conda.exe 路径推导 CONDA_ROOT（Scripts\conda.exe → 上两级）
if (-not $env:CONDA_ROOT) {
    $env:CONDA_ROOT = Split-Path (Split-Path $_condaExe -Parent) -Parent
}
Write-Host "  conda: $_condaExe (CONDA_ROOT=$env:CONDA_ROOT)"

# 用 conda run -n gmr 检查 conda-pack 是否已安装在 gmr 环境
# 注意：$ErrorActionPreference="Stop" 时原生命令的 stderr 会触发终止，
# conda run 在 import 失败时把 traceback 写到 stderr，需临时降级。
$prevEAP = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$_has_pack = $(& $_condaExe run -n gmr python -c "import conda_pack; print('yes')" 2>&1 | Where-Object { $_ -is [string] }) -join "`n"
$_has_pack = ($_has_pack -split "`n" | Where-Object { $_ -eq "yes" }).Count -gt 0
$ErrorActionPreference = $prevEAP

if (-not $_has_pack) {
    Write-Host "  conda-pack 未安装，安装到 gmr 环境（不装 base 避免 setuptools 冲突）..."
    $ErrorActionPreference = "Continue"
    & $_condaExe install -y -n gmr -c conda-forge conda-pack 2>&1 | Out-Null
    $installOk = $LASTEXITCODE -eq 0
    if (-not $installOk) {
        Write-Host "  conda install 失败，尝试 pip install conda-pack ..."
        & $_condaExe run -n gmr pip install conda-pack 2>&1 | Out-Null
    }
    $ErrorActionPreference = $prevEAP
}

Write-Host "[2/7] 清理并创建暂存目录 $Stage"
if (Test-Path $Stage) { Remove-Item -Recurse -Force $Stage }
New-Item -ItemType Directory -Path $Stage | Out-Null

Write-Host "[3/7] 打包 gmr conda 环境"
$envZip = Join-Path $Stage "env.zip"
# conda-pack 的命令行入口是 conda-pack.exe（在 gmr 环境的 Scripts/ 下），
# 不是 python -m conda_pack（该包没有 __main__.py）。
# 直接用 conda-pack.exe 绝对路径调用（不通过 conda run），避免 conda run 传递 exit code 的问题。
$_condaPackExe = Join-Path $env:CONDA_ROOT "envs\gmr\Scripts\conda-pack.exe"
if (-not (Test-Path $_condaPackExe)) {
    # 回退：从 conda.exe 推导 CONDA_ROOT
    $_root = Split-Path (Split-Path $_condaExe -Parent) -Parent
    $_condaPackExe = Join-Path $_root "envs\gmr\Scripts\conda-pack.exe"
}
if (-not (Test-Path $_condaPackExe)) { throw "conda-pack.exe 未找到，请确认 conda-pack 已安装到 gmr 环境" }
$ErrorActionPreference = "Continue"
& $_condaPackExe -n gmr -o $envZip --force --ignore-missing-files --ignore-editable-packages 2>&1 | ForEach-Object { Write-Host "  $_" }
$packExit = $LASTEXITCODE
$ErrorActionPreference = $prevEAP
if ($packExit -ne 0 -or -not (Test-Path $envZip)) {
    throw "conda-pack 打包失败 (exit $packExit)，env.zip 未生成"
}
$envDir = Join-Path $Stage "env"
New-Item -ItemType Directory -Path $envDir | Out-Null
Expand-Archive -Path $envZip -DestinationPath $envDir -Force
Remove-Item $envZip

Write-Host "[4/7] 写入 .pack_source_root 并创建 activate.d 钩子"
Set-Content -Path (Join-Path $envDir ".pack_source_root") -Value $GmrRoot

# 创建 activate.d 钩子：每次激活时自动重写 editable 路径（finder MAPPING + direct_url.json）
# activate.bat 只执行 activate.d/*.bat（不执行 .ps1），
# 所以需要 .bat 包装器调用 .ps1。.bat 纯 ASCII 避免编码问题。
$actDir = Join-Path $envDir "etc\conda\activate.d"
New-Item -ItemType Directory -Path $actDir -Force | Out-Null

$psHook = @'
# Editable path rewriter: rewrite source paths from pack machine to local
$packRoot = Join-Path $env:CONDA_PREFIX ".pack_source_root"
if (-not (Test-Path $packRoot)) { return }
$old = (Get-Content $packRoot -Raw).Trim()
$new = Join-Path $env:CONDA_PREFIX "..\gmr" | Resolve-Path -ErrorAction SilentlyContinue
if (-not $new) { return }
$new = $new.Path
$site = Join-Path $env:CONDA_PREFIX "Lib\site-packages"
if (-not (Test-Path $site)) { return }

# 1) Rewrite __editable___*_finder.py MAPPING paths
# 用 .Replace()（字面替换）而非 -replace（正则），避免 [regex]::Escape() 在替换串中多加反斜杠
Get-ChildItem (Join-Path $site "__editable___*_finder.py") -ErrorAction SilentlyContinue | ForEach-Object {
    $c = Get-Content $_.FullName -Raw
    $c = $c.Replace($old.Replace('\', '\\'), $new.Replace('\', '\\'))
    $c = $c.Replace($old.Replace('\', '/'), $new.Replace('\', '/'))
    [System.IO.File]::WriteAllText($_.FullName, $c, [System.Text.UTF8Encoding]::new($false))
}

# 2) Rewrite direct_url.json (JSON 中路径用 \\ 转义)
Get-ChildItem (Join-Path $site "*.dist-info\direct_url.json") -ErrorAction SilentlyContinue | ForEach-Object {
    $data = Get-Content $_.FullName -Raw | ConvertFrom-Json
    $oldUri = ([System.Uri]$old).AbsoluteUri.TrimEnd('/')
    $newUri = ([System.Uri]$new).AbsoluteUri.TrimEnd('/')
    if ($data.url -eq $oldUri -or $data.url.StartsWith($oldUri + '/')) {
        $data.url = $data.url.Replace($oldUri, $newUri)
        [System.IO.File]::WriteAllText($_.FullName, ($data | ConvertTo-Json -Depth 10), [System.Text.UTF8Encoding]::new($false))
    }
}

# 3) Remove legacy egg-link files
Get-ChildItem (Join-Path $site "*.egg-link") -ErrorAction SilentlyContinue | Remove-Item -Force
'@
$psHookPath = Join-Path $actDir "zz_editable_rewrite.ps1"
# UTF-8 BOM 编码，确保 PowerShell 5.1 正确读取
$utf8Bom = [System.Text.Encoding]::UTF8.GetPreamble()
$psHookBytes = $utf8Bom + [System.Text.Encoding]::UTF8.GetBytes($psHook)
[System.IO.File]::WriteAllBytes($psHookPath, $psHookBytes)

# .bat 包装器：activate.bat 只执行 activate.d/*.bat（不执行 .ps1），
# 所以需要一个 .bat 来调用 .ps1。纯 ASCII，不含中文，避免编码问题。
$batHook = '@echo off' + "`r`n" + 'powershell -NoProfile -ExecutionPolicy Bypass -File "%CONDA_PREFIX%\etc\conda\activate.d\zz_editable_rewrite.ps1"' + "`r`n"
$batHookPath = Join-Path $actDir "zz_editable_rewrite.bat"
[System.IO.File]::WriteAllText($batHookPath, $batHook, [System.Text.Encoding]::ASCII)

Write-Host "[5/7] 复制 GMR 仓库到 $Stage\gmr"
$gmrStage = Join-Path $Stage "gmr"
$bodyModels = Join-Path $GmrRoot "assets\body_models"
$sonicLib = Join-Path $GmrRoot "sonic_sim\lib"
& robocopy $GmrRoot $gmrStage /MIR `
  /XD .git __pycache__ .venv .idea node_modules build dist .workbuddy .vscode .pytest_cache output $sonicLib $bodyModels `
  /XF *.pyc MUJOCO_LOG.TXT _run_output.txt tmpajb39ryj.xml *.so *.so.*
if ($LASTEXITCODE -ge 8) { throw "robocopy 失败 exit $LASTEXITCODE" }

Write-Host "[6/7] 写入启动器"
Copy-Item (Join-Path $GmrRoot "scripts\run_teleop_studio.ps1") (Join-Path $Stage "run_teleop_studio.ps1")
$sourceRevision = "unknown"
if (Get-Command git -ErrorAction SilentlyContinue) {
    $ErrorActionPreference = "Continue"
    $revision = & git -C $GmrRoot rev-parse HEAD 2>$null
    $gitExit = $LASTEXITCODE
    $ErrorActionPreference = $prevEAP
    if ($gitExit -eq 0) { $sourceRevision = $revision }
}
@"
Teleop Studio Windows
source_revision=$sourceRevision
preview_defaults_ms=default:400,low_latency:100
history_window_frames=2
SMPL-X body models are not bundled; Moretion teleoperation does not need them.
"@ | Set-Content -Encoding UTF8 (Join-Path $Stage "PACKAGE_INFO.txt")

Write-Host "[7/7] 生成分发包 dist\$OutName.zip"
$distDir = Join-Path $GmrRoot "dist"
if (-not (Test-Path $distDir)) { New-Item -ItemType Directory -Path $distDir | Out-Null }
Compress-Archive -Path (Join-Path $Stage "*") -DestinationPath (Join-Path $distDir "$OutName.zip") -Force
$archiveHash = (Get-FileHash -Algorithm SHA256 (Join-Path $distDir "$OutName.zip")).Hash.ToLowerInvariant()
"$archiveHash  $OutName.zip" | Set-Content -Encoding ASCII (Join-Path $distDir "$OutName.zip.sha256")
Write-Host "完成：$distDir\$OutName.zip"
Write-Host "安装：解压后运行 run_teleop_studio.ps1，路径全自动确定"
