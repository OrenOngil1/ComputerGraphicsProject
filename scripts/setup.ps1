#Requires -Version 5.1
<#
.SYNOPSIS
    Install the build prerequisites for Visual SLAMMER on Windows.

.DESCRIPTION
    Unlike setup.sh (which installs the libraries themselves via apt/brew), this
    script only ensures the two PREREQUISITES exist:
      1. The MSVC C++ compiler (Visual Studio Build Tools).
      2. vcpkg, the package manager that fetches GLFW / OpenCV / etc.

    The libraries are installed automatically by vcpkg the first time you
    configure the project with the vcpkg toolchain file -- see the README.

    Safe to re-run: each step is skipped if it is already satisfied.
#>
[CmdletBinding()]
param(
    # Where to clone vcpkg if it isn't already present. Honors an existing
    # VCPKG_ROOT; otherwise defaults to C:\vcpkg.
    [string]$VcpkgRoot = $(if ($env:VCPKG_ROOT) { $env:VCPKG_ROOT } else { "C:\vcpkg" })
)

$ErrorActionPreference = "Stop"

# ── 1. MSVC C++ compiler ──────────────────────────────────────
# Probe via vswhere for the actual VC++ tools component, not just any VS install.
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$hasMsvc = $false
if (Test-Path $vswhere) {
    $vc = & $vswhere -products * `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath
    if ($vc) { $hasMsvc = $true }
}

if ($hasMsvc) {
    Write-Host "[ok] MSVC C++ compiler found."
} else {
    Write-Host "[!]  MSVC C++ compiler (Visual Studio Build Tools) not found."
    $answer = Read-Host "     Install it now via winget? (~3-5 GB download) [y/N]"
    if ($answer -match '^[Yy]') {
        if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
            Write-Error "winget is unavailable. Install Build Tools manually: https://aka.ms/vs/17/release/vs_BuildTools.exe"
        }
        Write-Host "     Installing Visual Studio Build Tools (this will take a while)..."
        winget install --id Microsoft.VisualStudio.2022.BuildTools -e --override "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
        Write-Host "[ok] Build Tools installed. Build from the 'Developer PowerShell for VS 2022' so cl.exe is on PATH."
    } else {
        Write-Host "     Skipped. Install it yourself, then re-run this script:"
        Write-Host '       winget install --id Microsoft.VisualStudio.2022.BuildTools -e --override "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"'
    }
}

# ── 2. vcpkg package manager ──────────────────────────────────
if (Test-Path "$VcpkgRoot\vcpkg.exe") {
    Write-Host "[ok] vcpkg found at $VcpkgRoot."
} else {
    Write-Host "[..] vcpkg not found; cloning into $VcpkgRoot ..."
    git clone https://github.com/microsoft/vcpkg $VcpkgRoot
    & "$VcpkgRoot\bootstrap-vcpkg.bat"
    Write-Host "[ok] vcpkg bootstrapped at $VcpkgRoot."
}

# ── Done: hand off the toolchain path for the configure step ───
$toolchain = "$VcpkgRoot/scripts/buildsystems/vcpkg.cmake" -replace '\\', '/'
Write-Host ""
Write-Host "Prerequisites ready. Configure the project with:"
Write-Host "  cmake -B build -DCMAKE_TOOLCHAIN_FILE=$toolchain"
Write-Host "(vcpkg installs GLFW/OpenCV on that first configure -- see the README.)"
