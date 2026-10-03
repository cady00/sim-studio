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

if (Test-Path $OutDir) { Remove-Item -Recurse -Force $OutDir }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
Copy-Item $exe (Join-Path $OutDir "simstudio.exe")
Copy-Item "LICENSE.txt" $OutDir
Copy-Item "THIRD_PARTY_LICENSES.md" $OutDir
Copy-Item "README.md" $OutDir
if (Test-Path "samples") {
    Copy-Item "samples" (Join-Path $OutDir "samples") -Recurse
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
            $source = Join-Path $binDir $name
            if (Test-Path $source) {
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

$zip = Join-Path "dist" "SimStudio-0.1.0-win64.zip"
if (Test-Path $zip) { Remove-Item $zip }
Compress-Archive -Path $OutDir -DestinationPath $zip
Write-Host "Wrote $zip"

$makensis = Get-Command makensis -ErrorAction SilentlyContinue
if ($makensis) {
    & makensis "/DSTAGE=$OutDir" "packaging\simstudio.nsi"
} else {
    Write-Host "makensis not found; zip only."
}
