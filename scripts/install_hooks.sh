#!/usr/bin/env bash
set -euo pipefail

# 保证在仓库根目录执行
REPO_ROOT=$(git rev-parse --show-toplevel)
cd "$REPO_ROOT"

HOOK_SCRIPTS=(
    "scripts/pre-commit-clang-format.sh"
    "scripts/pre-commit-spdlog-check.sh"
)
HOOK_DST=".git/hooks/pre-commit"

echo "🔧 Installing git pre-commit hook..."

# 1. 基本检查
if [ ! -d ".git" ]; then
    echo "❌ Not a git repository"
    exit 1
fi

for script in "${HOOK_SCRIPTS[@]}"; do
    if [ ! -f "$script" ]; then
        echo "❌ Hook source not found: $script"
        exit 1
    fi
done

# 2. 确保脚本可执行
for script in "${HOOK_SCRIPTS[@]}"; do
    chmod +x "$script"
done

# 3. 创建 hooks 目录（极端情况下）
mkdir -p .git/hooks

touch "$HOOK_DST"
rm "$HOOK_DST"
# 4. 备份已有 hook（不覆盖用户已有逻辑）
if [ -e "$HOOK_DST" ] && [ ! -L "$HOOK_DST" ]; then
    echo "⚠️  Existing pre-commit hook found, backing up to pre-commit.bak"
    mv "$HOOK_DST" "$HOOK_DST.bak"
fi

# 5. 生成组合 hook 脚本
HOOK_CONTENT="#!/usr/bin/env bash
set -euo pipefail

# 此文件由 install_hooks.sh 自动生成
# 不要直接编辑此文件，修改 scripts/pre-commit-*.sh

"
for script in "${HOOK_SCRIPTS[@]}"; do
    HOOK_CONTENT+="source \"$script\"\n"
done

echo -e "$HOOK_CONTENT" > "$HOOK_DST"
chmod +x "$HOOK_DST"

echo "✅ pre-commit hook installed successfully"
