# PowerShell script to test Tractobots setup from Windows
# Run this as: .\test_setup.ps1

Write-Host "🚜 TRACTOBOTS SETUP TEST" -ForegroundColor Green
Write-Host "Testing your development environment..." -ForegroundColor Yellow

# Test WSL availability
Write-Host "`n1. Testing WSL availability..." -ForegroundColor Cyan
try {
    $wslTest = wsl --list --verbose
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ WSL is available" -ForegroundColor Green
        Write-Host "WSL Distributions:" -ForegroundColor Yellow
        Write-Host $wslTest
    } else {
        Write-Host "❌ WSL not properly configured" -ForegroundColor Red
    }
} catch {
    Write-Host "❌ WSL not available: $_" -ForegroundColor Red
}

# Test workspace structure
Write-Host "`n2. Testing workspace structure..." -ForegroundColor Cyan
$requiredDirs = @("src", ".vscode", "src\tractobots_bringup", "src\tractobots_navigation")
foreach ($dir in $requiredDirs) {
    if (Test-Path $dir) {
        Write-Host "✅ Found: $dir" -ForegroundColor Green
    } else {
        Write-Host "❌ Missing: $dir" -ForegroundColor Red
    }
}

# Test VS Code files
Write-Host "`n3. Testing VS Code configuration..." -ForegroundColor Cyan
$vscodeFiles = @(".vscode\settings.json", ".vscode\tasks.json", ".vscode\launch.json")
foreach ($file in $vscodeFiles) {
    if (Test-Path $file) {
        Write-Host "✅ Found: $file" -ForegroundColor Green
    } else {
        Write-Host "❌ Missing: $file" -ForegroundColor Red
    }
}

# Test helper scripts
Write-Host "`n4. Testing helper scripts..." -ForegroundColor Cyan
$scripts = @("quick_setup.sh", "verify_setup.py", "test_imports.py")
foreach ($script in $scripts) {
    if (Test-Path $script) {
        Write-Host "✅ Found: $script" -ForegroundColor Green
    } else {
        Write-Host "❌ Missing: $script" -ForegroundColor Red
    }
}

# Test ROS2 in WSL
Write-Host "`n5. Testing ROS2 in WSL..." -ForegroundColor Cyan
try {
    $ros2Test = wsl bash -c "source /opt/ros/humble/setup.bash 2>/dev/null && echo 'ROS2 sourced successfully' && ros2 --version 2>/dev/null"
    if ($LASTEXITCODE -eq 0 -and $ros2Test -like "*ROS2*") {
        Write-Host "✅ ROS2 is available in WSL" -ForegroundColor Green
        Write-Host $ros2Test
    } else {
        Write-Host "❌ ROS2 not properly configured in WSL" -ForegroundColor Red
        Write-Host "Output: $ros2Test"
    }
} catch {
    Write-Host "❌ Could not test ROS2 in WSL: $_" -ForegroundColor Red
}

# Run Python verification in WSL
Write-Host "`n6. Running Python verification in WSL..." -ForegroundColor Cyan
try {
    $pythonTest = wsl bash -c "cd ~/ros2_tractobots 2>/dev/null || cd /mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots && python3 verify_setup.py"
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ Python verification completed" -ForegroundColor Green
    } else {
        Write-Host "⚠️ Python verification had issues (check output above)" -ForegroundColor Yellow
    }
} catch {
    Write-Host "❌ Could not run Python verification: $_" -ForegroundColor Red
}

Write-Host "`n🎯 SUMMARY:" -ForegroundColor Green
Write-Host "- If all tests pass, your environment is ready!" -ForegroundColor Yellow
Write-Host "- Open VS Code and try building with Ctrl+Shift+P → Tasks → '🚜 Build Tractobots'" -ForegroundColor Yellow
Write-Host "- Any Python import warnings in VS Code are cosmetic only" -ForegroundColor Yellow
Write-Host "- Use WSL terminal for ROS2 commands: wsl" -ForegroundColor Yellow

Write-Host "`n📚 DOCUMENTATION:" -ForegroundColor Green
Write-Host "- SETUP_STATUS.md - Complete setup summary" -ForegroundColor Cyan
Write-Host "- VSCODE_BUILD_GUIDE.md - How to build and run" -ForegroundColor Cyan
Write-Host "- PYTHON_IMPORTS_FIX.md - Import warning explanation" -ForegroundColor Cyan

Read-Host "`nPress Enter to exit"
