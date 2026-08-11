param(
    [string]$StandardCad = "",
    [string]$AlignedScan = "",
    [string]$OutputDirectory = ""
)
$ErrorActionPreference = "Stop"
$workspace = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$StandardCad = if ($StandardCad) { $StandardCad } else { Join-Path $workspace "data\output\sampling\vg1500040104a_sampled.ply" }
$AlignedScan = if ($AlignedScan) { $AlignedScan } else { Join-Path $workspace "data\output\registration\zhujian_1seg_aligned.ply" }
$OutputDirectory = if ($OutputDirectory) { $OutputDirectory } else { Join-Path $workspace "data\output\inspection\run" }
$executable = Join-Path $workspace "build\inspection\Release\cadinspect_ply_test.exe"
foreach ($path in @($executable, $StandardCad, $AlignedScan)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Required file not found: $path" }
}
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
& $executable $StandardCad $AlignedScan $OutputDirectory
if ($LASTEXITCODE -ne 0) { throw "Inspection test failed with exit code $LASTEXITCODE" }
