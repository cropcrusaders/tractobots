# 🛠️ Fixing ShellCheck Extension Error

## Problem
ShellCheck extension error: `'shellcheck' is not recognized as an internal or external command`

## 🚀 Solutions (Choose One)

### Option 1: Install ShellCheck in WSL (Recommended)
```bash
# In WSL Ubuntu terminal
sudo apt update
sudo apt install shellcheck

# Verify installation
shellcheck --version
```

### Option 2: Use Windows ShellCheck
```powershell
# Install via Chocolatey (if you have it)
choco install shellcheck

# Or download from: https://github.com/koalaman/shellcheck/releases
# Extract shellcheck.exe to a folder in your PATH
```

### Option 3: Disable ShellCheck (Quick Fix)
Replace your `.vscode/settings.json` with the fixed version:

```bash
# In project root
cp .vscode/settings_fixed.json .vscode/settings.json
```

Or manually add to your VS Code settings:
```json
{
    "shellcheck.enable": false,
    "shellcheck.useWSL": true
}
```

### Option 4: Configure ShellCheck for WSL
Add to `.vscode/settings.json`:
```json
{
    "shellcheck.useWSL": true,
    "shellcheck.executablePath": "wsl shellcheck"
}
```

## 🎯 Recommended Solution

**For Tractobots development**, use Option 1 (Install in WSL):

1. **Open WSL Ubuntu terminal**
2. **Install ShellCheck:**
   ```bash
   sudo apt update && sudo apt install shellcheck
   ```
3. **Update VS Code settings:**
   ```bash
   cd /mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots
   cp .vscode/settings_fixed.json .vscode/settings.json
   ```
4. **Reload VS Code:** Ctrl+Shift+P → "Developer: Reload Window"

## ✅ Verification

After applying the fix:

1. **Open a shell script** (like `quick_setup.sh`)
2. **Check for linting:** Should see syntax highlighting without errors
3. **Test build:** Run "🚜 Build Tractobots" task

## 🔧 Alternative Extensions

If ShellCheck continues to cause issues, consider these alternatives:

1. **Shell Format** (foxundermoon.shell-format) - For formatting only
2. **Bash IDE** (mads-hartmann.bash-ide-vscode) - Full bash support
3. **Bash Debug** (rogalmic.bash-debug) - For debugging scripts

## 💡 Why This Happens

- ShellCheck is a Linux tool for shell script linting
- VS Code on Windows can't find it unless properly configured
- WSL provides the Linux environment where ShellCheck works best
- Your shell scripts are designed for Linux/WSL execution anyway

## 🚜 Back to Building Your Tractor

Once fixed, you can continue with:
```bash
# Build your autonomous tractor
./quick_setup.sh

# Or use VS Code tasks
# Ctrl+Shift+B → "🚜 Build Tractobots"
```
