# Stage a Windows build of simstudio.exe with the Qt and OCCT DLLs it needs.
# windeployqt is not in the vcpkg manifest (that feature is Windows-only and
# would break a Linux manifest install), so this script copies dependents itself.
param(
    [string]$BuildDir = "out/build/windows-msvc",
    [string]$Config = "Release",
    [string]$OutDir = "dist/SimStudio"
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $root

$candidates = @(
    (Join-Path $BuildDir "src/app/$Config/simstudio.exe"),
    (Join-Path $BuildDir "src/app/simstudio.exe")
)
$exe = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $exe) {
    throw "simstudio.exe not found under $BuildDir"
}

$installed = Join-Path $BuildDir "vcpkg_installed/x64-windows-release"
if (-not (Test-Path $installed)) {
    $installed = Join-Path $BuildDir "vcpkg_installed/x64-windows"
}
$binDir = Join-Path $installed "bin"
$pluginRoot = Join-Path $installed "Qt6/plugins"
if (-not (Test-Path $pluginRoot)) {
    $pluginRoot = Join-Path $installed "plugins"
}
$qtBin = $null
if ($env:QT_ROOT_DIR) {
    $qtBin = Join-Path $env:QT_ROOT_DIR "bin"
    if (-not (Test-Path $pluginRoot)) {
        $qtPlugins = Join-Path $env:QT_ROOT_DIR "plugins"
        if (Test-Path $qtPlugins) { $pluginRoot = $qtPlugins }
    }
}
function Find-RuntimeDll([string]$name) {
    $fromVcpkg = Join-Path $binDir $name
    if (Test-Path $fromVcpkg) { return $fromVcpkg }
    if ($qtBin) {
        $fromQt = Join-Path $qtBin $name
        if (Test-Path $fromQt) { return $fromQt }
    }
    return $null
}

if (Test-Path $OutDir) { Remove-Item -Recurse -Force $OutDir }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
Copy-Item $exe (Join-Path $OutDir "simstudio.exe")
$windeploy = $null
if ($qtBin) {
    $candidate = Join-Path $qtBin "windeployqt.exe"
    if (Test-Path $candidate) { $windeploy = $candidate }
}
if (-not $windeploy) {
    $cmd = Get-Command windeployqt -ErrorAction SilentlyContinue
    if ($cmd) { $windeploy = $cmd.Source }
}
if ($windeploy) {
    & $windeploy --release --no-translations --compiler-runtime (Join-Path $OutDir "simstudio.exe")
}
Copy-Item "LICENSE.txt" $OutDir
Copy-Item "THIRD_PARTY_LICENSES.md" $OutDir
Copy-Item "README.md" $OutDir
$brand = Join-Path $root "resources/branding"
if (Test-Path $brand) {
    Copy-Item $brand (Join-Path $OutDir "branding") -Recurse
    Copy-Item (Join-Path $brand "app.ico") (Join-Path $OutDir "simstudio.ico")
    Copy-Item (Join-Path $brand "tray.ico") (Join-Path $OutDir "tray.ico")
}
if (Test-Path "samples") {
    Copy-Item "samples" (Join-Path $OutDir "samples") -Recurse
}
$manual = Join-Path $root "docs/manual"
if (Test-Path $manual) {
    Copy-Item $manual (Join-Path $OutDir "manual") -Recurse
}
$meshNote = Join-Path $root "docs/MESH_OFFLINE.txt"
if (Test-Path $meshNote) {
    Copy-Item $meshNote (Join-Path $OutDir "网格离线开关.txt")
}

$dumpbin = Get-ChildItem "C:\Program Files\Microsoft Visual Studio" -Recurse -Filter dumpbin.exe -ErrorAction SilentlyContinue |
    Where-Object { $_.FullName -match "Hostx64\\x64" } |
    Select-Object -First 1 -ExpandProperty FullName
if (-not $dumpbin) {
    throw "dumpbin.exe not found. Run this from a Visual Studio 2022 developer environment."
}

$seen = @{}
$queue = New-Object System.Collections.Generic.Queue[string]
$queue.Enqueue((Resolve-Path $exe).Path)
while ($queue.Count -gt 0) {
    $current = $queue.Dequeue()
    $lines = & $dumpbin /dependents $current
    foreach ($line in $lines) {
        if ($line -match "^\s+(\S+\.dll)\s*$") {
            $name = $Matches[1]
            if ($seen.ContainsKey($name)) { continue }
            $seen[$name] = $true
            $source = Find-RuntimeDll $name
            if ($source) {
                Copy-Item $source (Join-Path $OutDir $name)
                $queue.Enqueue((Join-Path $OutDir $name))
            }
        }
    }
}

$systemDlls = @("vcruntime140.dll", "vcruntime140_1.dll", "msvcp140.dll", "msvcp140_1.dll", "msvcp140_2.dll")
$redist = Get-ChildItem "C:\Program Files\Microsoft Visual Studio" -Recurse -Filter "vcruntime140.dll" -ErrorAction SilentlyContinue |
    Where-Object { $_.FullName -match "\\x64\\" } |
    Select-Object -First 1
if ($redist) {
    $redistDir = Split-Path $redist.FullName
    foreach ($name in $systemDlls) {
        $source = Join-Path $redistDir $name
        if ((Test-Path $source) -and -not (Test-Path (Join-Path $OutDir $name))) {
            Copy-Item $source (Join-Path $OutDir $name)
        }
    }
}

$pluginDest = Join-Path $OutDir "plugins"
foreach ($rel in @("platforms", "styles", "imageformats")) {
    $source = Join-Path $pluginRoot $rel
    if (Test-Path $source) {
        Copy-Item $source (Join-Path $pluginDest $rel) -Recurse
    }
}
@"
[Paths]
Plugins = plugins
"@ | Set-Content -Encoding ascii (Join-Path $OutDir "qt.conf")

# Embedded CPython looks for Lib/ (and python3xx._pth) beside the executable.
$pythonLib = Join-Path $installed "tools/python3/Lib"
$pythonDlls = Join-Path $installed "tools/python3/DLLs"
if (Test-Path $pythonLib) {
    Copy-Item $pythonLib (Join-Path $OutDir "Lib") -Recurse
}
if (Test-Path $pythonDlls) {
    $dllDest = Join-Path $OutDir "DLLs"
    New-Item -ItemType Directory -Force -Path $dllDest | Out-Null
    Copy-Item (Join-Path $pythonDlls "*") $dllDest -Recurse -Force
    Get-ChildItem $dllDest -Filter *.pyd -Recurse | ForEach-Object {
        $queue.Enqueue($_.FullName)
    }
    while ($queue.Count -gt 0) {
        $current = $queue.Dequeue()
        $lines = & $dumpbin /dependents $current
        foreach ($line in $lines) {
            if ($line -match "^\s+(\S+\.dll)\s*$") {
                $name = $Matches[1]
                if ($seen.ContainsKey($name)) { continue }
                $seen[$name] = $true
                $source = Find-RuntimeDll $name
                if ($source) {
                    Copy-Item $source (Join-Path $OutDir $name)
                    $queue.Enqueue((Join-Path $OutDir $name))
                }
            }
        }
    }
}
$pyDll = Get-ChildItem $OutDir -Filter "python3*.dll" |
    Where-Object { $_.Name -match "^python3\d+\.dll$" } |
    Select-Object -First 1
if ($pyDll) {
    $pthName = [System.IO.Path]::GetFileNameWithoutExtension($pyDll.Name) + "._pth"
    @"
Lib
DLLs
.
import site
"@ | Set-Content -Encoding ascii (Join-Path $OutDir $pthName)
}

$version = "0.2.0"
$versionHeader = Join-Path $root "src/core/include/sim/core/Version.hpp"
if (Test-Path $versionHeader) {
    $versionText = Get-Content $versionHeader -Raw
    if ($versionText -match 'kVersion = "([^"]+)"') {
        $version = $Matches[1]
    }
}
$zip = Join-Path "dist" "SimStudio-$version-win64.zip"
if (Test-Path $zip) { Remove-Item $zip }
Compress-Archive -Path $OutDir -DestinationPath $zip
Write-Host "Wrote $zip"

$makensis = Get-Command makensis -ErrorAction SilentlyContinue
if ($makensis) {
    & makensis "/DSTAGE=$OutDir" "packaging\simstudio.nsi"
} else {
    Write-Host "makensis not found; zip only."
}
