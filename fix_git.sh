#!/bin/bash

PROJECT_DIR="/home/edg5/capstone_project/Marine_Docking_Platform"
cd "$PROJECT_DIR" || { echo "Project directory not found!"; exit 1; }

echo "=========================================="
echo "      GIT DIAGNOSTIC & REPAIR TOOL        "
echo "=========================================="

# 1. Fix Identity
echo "[1] Setting User Identity..."
git config user.name "dgranda2022"
git config user.email "dgranda2022@fau.edu"
echo "    User:  $(git config user.name)"
echo "    Email: $(git config user.email)"

# 2. Check for Nested Repository Mistake
# (Common if someone ran 'git clone' inside the folder)
echo ""
echo "[2] Checking for nested repositories..."
if [ -d "$PROJECT_DIR/Marine_Docking_Platform" ]; then
    echo "    [WARNING] Found a nested 'Marine_Docking_Platform' folder!"
    echo "    This happens when 'git clone' is run inside an existing repo."
    echo "    Recommendation: Check if it contains important files, then remove it."
else
    echo "    [OK] No nested repositories found."
fi

# 3. Fix Remote
echo ""
echo "[3] Verifying Remote Origin..."
TARGET_URL="https://github.com/dgranda2022/Marine_Docking_Platform.git"
CURRENT_URL=$(git remote get-url origin 2>/dev/null)

if [ "$CURRENT_URL" != "$TARGET_URL" ]; then
    echo "    [FIX] Resetting remote URL..."
    if [ -z "$CURRENT_URL" ]; then
        git remote add origin "$TARGET_URL"
    else
        git remote set-url origin "$TARGET_URL"
    fi
else
    echo "    [OK] Remote URL is correct."
fi

# 4. Network Check
echo ""
echo "[4] Testing Connection..."
if ping -c 1 8.8.8.8 &> /dev/null; then
    echo "    [OK] Internet connection active."
    echo "    Fetching from GitHub..."
    git fetch origin
    if [ $? -eq 0 ]; then
        echo "    [SUCCESS] Repository is healthy and connected."
    else
        echo "    [FAIL] Internet is on, but GitHub refused connection."
    fi
else
    echo "    [FAIL] No Internet access."
    echo "    ACTION: Run the 'route add' command from the Operations Manual."
fi

echo "=========================================="