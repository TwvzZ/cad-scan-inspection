param(
    [string]$SourceScan = "",

    [string]$TargetCad = "",

    [string]$OutputAligned = "",

    [ValidateSet("single", "cascade", "ensemble")]
    [string]$Mode = "cascade",

    [ValidateSet("initial", "pca", "fpfh", "all")]
    [string]$Strategy = "all",

    [double]$MaxDistance = 5.0,
    [UInt32]$Iterations = 100,
    [double]$VoxelSize = 1.0,
    [double]$FeatureVoxelSize = 5.0,
    [string]$InitialMatrix = "",
    [ValidateSet("fast", "balanced", "accuracy")]
    [string]$IcpPreset = "balanced",
    [UInt32]$RansacMaxIterations = 50000,
    [UInt32]$RansacAttempts = 4,
    [UInt32]$MaxCandidates = 4,
    [UInt32]$MaxRefinedCandidates = 2,
    [string]$PclRoot = ""
)

$workspaceRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$PclRoot = if ($PclRoot) { $PclRoot } else { Join-Path $workspaceRoot "third_party\pcl" }
$SourceScan = if ($SourceScan) { $SourceScan } else { Join-Path $workspaceRoot "data\input\scan\zhujian_1seg.ply" }
$TargetCad = if ($TargetCad) { $TargetCad } else { Join-Path $workspaceRoot "data\output\sampling\vg1500040104a_sampled.ply" }
$OutputAligned = if ($OutputAligned) { $OutputAligned } else { Join-Path $workspaceRoot "data\output\registration\zhujian_1seg_aligned.ply" }
$executable = Join-Path $workspaceRoot "build\registration\Release\cadreg_test.exe"
if (-not (Test-Path -LiteralPath $executable)) {
    throw "Registration test executable not found: $executable"
}
if (-not (Test-Path -LiteralPath $PclRoot)) {
    throw "PCL root not found: $PclRoot"
}
if ($RansacMaxIterations -eq 0 -or $RansacAttempts -eq 0 -or
    $MaxCandidates -eq 0 -or $MaxRefinedCandidates -eq 0) {
    throw "RANSAC and candidate count parameters must be greater than zero"
}
if ($MaxRefinedCandidates -gt $MaxCandidates) {
    throw "MaxRefinedCandidates cannot exceed MaxCandidates"
}

$runtimeDirectories = @((Split-Path -Parent $executable))
$runtimeDirectories += Get-ChildItem -LiteralPath $PclRoot `
    -Filter *.dll -File -Recurse -ErrorAction SilentlyContinue |
    ForEach-Object { $_.DirectoryName } |
    Sort-Object -Unique
$env:PATH = (($runtimeDirectories | Select-Object -Unique) -join ";") +
    ";" + $env:PATH

$arguments = @(
    $SourceScan,
    $TargetCad,
    $OutputAligned,
    $Mode,
    $Strategy,
    $MaxDistance.ToString([Globalization.CultureInfo]::InvariantCulture),
    $Iterations.ToString([Globalization.CultureInfo]::InvariantCulture),
    $VoxelSize.ToString([Globalization.CultureInfo]::InvariantCulture),
    $FeatureVoxelSize.ToString([Globalization.CultureInfo]::InvariantCulture),
    $(if ($InitialMatrix) { $InitialMatrix } else { "-" }),
    $RansacMaxIterations.ToString(
        [Globalization.CultureInfo]::InvariantCulture),
    $RansacAttempts.ToString(
        [Globalization.CultureInfo]::InvariantCulture),
    $MaxCandidates.ToString(
        [Globalization.CultureInfo]::InvariantCulture),
    $MaxRefinedCandidates.ToString(
        [Globalization.CultureInfo]::InvariantCulture),
    $IcpPreset
)
if ($InitialMatrix) {
    if (-not (Test-Path -LiteralPath $InitialMatrix -PathType Leaf)) {
        throw "Initial matrix file not found: $InitialMatrix"
    }
}

& $executable @arguments

exit $LASTEXITCODE






