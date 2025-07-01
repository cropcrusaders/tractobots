# Tractobots Windows Setup Script
# Run this in PowerShell to set up WSL2 and get started

Write-Host "🚜 Tractobots Windows Setup" -ForegroundColor Green
Write-Host "===========================" -ForegroundColor Green

# Check if running as administrator
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")

if (-not $isAdmin) {
    Write-Host "⚠️  This script requires administrator privileges for WSL setup" -ForegroundColor Yellow
    Write-Host "Right-click PowerShell and 'Run as Administrator'" -ForegroundColor Yellow
    exit 1
}

# Check if WSL is already installed
$wslInstalled = Get-Command wsl -ErrorAction SilentlyContinue

if (-not $wslInstalled) {
    Write-Host "📦 Installing WSL2..." -ForegroundColor Blue
    wsl --install
    Write-Host "⚠️  Please restart your computer and run this script again" -ForegroundColor Yellow
    exit 0
}

# Check if Ubuntu 22.04 is available
$ubuntuInstalled = wsl -l -v | Select-String "Ubuntu-22.04"

if (-not $ubuntuInstalled) {
    Write-Host "📦 Installing Ubuntu 22.04..." -ForegroundColor Blue
    wsl --install -d Ubuntu-22.04
}

Write-Host "✅ WSL2 and Ubuntu 22.04 are ready!" -ForegroundColor Green
Write-Host ""
Write-Host "🚀 Next Steps:" -ForegroundColor Cyan
Write-Host "1. Open Ubuntu 22.04 from Start Menu" -ForegroundColor White
Write-Host "2. In Ubuntu terminal, run:" -ForegroundColor White
Write-Host "   cd /mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/tractobots" -ForegroundColor Yellow
Write-Host "   chmod +x install_ros2_humble.sh" -ForegroundColor Yellow
Write-Host "   ./install_ros2_humble.sh" -ForegroundColor Yellow
Write-Host "   chmod +x quick_setup.sh" -ForegroundColor Yellow
Write-Host "   ./quick_setup.sh" -ForegroundColor Yellow
Write-Host ""
Write-Host "💡 For GUI applications (like RViz), install VcXsrv X Server on Windows" -ForegroundColor Cyan
Write-Host "   Download from: https://sourceforge.net/projects/vcxsrv/" -ForegroundColor Yellow
Write-Host "   Then in Ubuntu: export DISPLAY=:0" -ForegroundColor Yellow

Write-Host ""
Write-Host "🎉 Ready to build your autonomous tractor!" -ForegroundColor Green
