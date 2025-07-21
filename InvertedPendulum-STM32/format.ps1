# Auto-format C++ code in Core/App directory using clang-format

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$TargetDir = Join-Path $ScriptDir "Core\App"

Write-Host "Formatting C++ files in $TargetDir..."

# Check if clang-format is available
if (-not (Get-Command clang-format -ErrorAction SilentlyContinue)) {
    Write-Error "Error: clang-format is not installed"
    Write-Host "Install with: winget install LLVM.LLVM"
    exit 1
}

# Find and format all .cpp and .hpp files
Get-ChildItem -Path $TargetDir -Recurse -Include "*.cpp", "*.hpp" | ForEach-Object {
    Write-Host "Formatting: $($_.FullName)"
    clang-format -i $_.FullName
}

Write-Host "Formatting complete!"