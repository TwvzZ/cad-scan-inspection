param(
    [string]$InputStep = "",

    [string]$OutputPly = "",

    [UInt64]$PointCount = 200000,
    [double]$VoxelSize = 0.5,
    [double]$LinearDeflection = 0.1,
    [double]$AngularDeflection = 10.0,
    [UInt32]$RelativeDeflection = 0,
    [UInt32]$ParallelMeshing = 1,
    [UInt32]$Seed = 1,
    [ValidateSet("all", "outer", "visible")]
    [string]$Mode = "outer",
    [ValidateSet("orthographic", "perspective")]
    [string]$Projection = "orthographic",
    [double]$ViewX = 0.0,
    [double]$ViewY = 0.0,
    [double]$ViewZ = -1.0,
    [ValidateRange(0.01, 89.99)]
    [double]$MaxIncidenceAngle = 75.0,
    [double]$CameraX = 0.0,
    [double]$CameraY = 0.0,
    [double]$CameraZ = 0.0,
    [double]$VisibilityTolerance = 0.0,
    [ValidateRange(1, 32)]
    [UInt32]$VisibilityOversampleFactor = 4
)

$ErrorActionPreference = "Stop"
$workspace = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$InputStep = if ($InputStep) { $InputStep } else { Join-Path $workspace "assets\\cad\\vg1500040104a.stp" }
$OutputPly = if ($OutputPly) { $OutputPly } else { Join-Path $workspace "data\\output\\sampling\\vg1500040104a_sampled_test.ply" }
$executable = Join-Path $workspace "build\sampling\Release\cadsample_test.exe"
$runtimeRoot = Join-Path $workspace "third_party\win-x64"
$occtBin = Join-Path $runtimeRoot "occt-7.9.3\bin"
$thirdPartyRoot = $runtimeRoot

if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
    throw "Sampling test executable not found: $executable"
}
if (-not (Test-Path -LiteralPath $InputStep -PathType Leaf)) {
    throw "STEP file not found: $InputStep"
}
if (-not (Test-Path -LiteralPath $occtBin -PathType Container)) {
    throw "OpenCASCADE runtime directory not found: $occtBin"
}
if (-not (Test-Path -LiteralPath $thirdPartyRoot -PathType Container)) {
    throw "OpenCASCADE third-party directory not found: $thirdPartyRoot"
}

$runtimeDirectories = @((Split-Path -Parent $executable), $occtBin, $runtimeRoot)
$runtimeDirectories += Get-ChildItem -LiteralPath $thirdPartyRoot `
    -Filter "*.dll" -File -Recurse |
    ForEach-Object { $_.DirectoryName } |
    Select-Object -Unique
$env:PATH = (($runtimeDirectories | Select-Object -Unique) -join ";") +
    ";" + $env:PATH

$arguments = @(
    $InputStep,
    $OutputPly,
    $PointCount.ToString([Globalization.CultureInfo]::InvariantCulture),
    $VoxelSize.ToString([Globalization.CultureInfo]::InvariantCulture),
    $LinearDeflection.ToString([Globalization.CultureInfo]::InvariantCulture),
    $AngularDeflection.ToString([Globalization.CultureInfo]::InvariantCulture),
    $RelativeDeflection.ToString([Globalization.CultureInfo]::InvariantCulture),
    $ParallelMeshing.ToString([Globalization.CultureInfo]::InvariantCulture),
    $Seed.ToString([Globalization.CultureInfo]::InvariantCulture),
    $Mode
)
if ($Mode -eq "visible") {
    $arguments += $Projection
    if ($Projection -eq "orthographic") {
        $arguments += $ViewX.ToString([Globalization.CultureInfo]::InvariantCulture)
        $arguments += $ViewY.ToString([Globalization.CultureInfo]::InvariantCulture)
        $arguments += $ViewZ.ToString([Globalization.CultureInfo]::InvariantCulture)
    } else {
        $arguments += $CameraX.ToString([Globalization.CultureInfo]::InvariantCulture)
        $arguments += $CameraY.ToString([Globalization.CultureInfo]::InvariantCulture)
        $arguments += $CameraZ.ToString([Globalization.CultureInfo]::InvariantCulture)
    }
    $arguments += $MaxIncidenceAngle.ToString(
        [Globalization.CultureInfo]::InvariantCulture)
}
$arguments += $VisibilityTolerance.ToString([Globalization.CultureInfo]::InvariantCulture)
$arguments += $VisibilityOversampleFactor.ToString([Globalization.CultureInfo]::InvariantCulture)

& $executable @arguments
if ($LASTEXITCODE -ne 0) {
    throw "Sampling test failed with process exit code $LASTEXITCODE"
}



