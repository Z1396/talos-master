#!/usr/bin/env bash
set -euo pipefail

CLANG_FORMAT=${CLANG_FORMAT:-clang-format}

# 检测终端是否支持颜色
if [ -t 1 ]; then
    RED=$(tput setaf 1 2>/dev/null || echo '')
    GREEN=$(tput setaf 2 2>/dev/null || echo '')
    CYAN=$(tput setaf 6 2>/dev/null || echo '')
    RESET=$(tput sgr0 2>/dev/null || echo '')
else
    RED=''
    GREEN=''
    CYAN=''
    RESET=''
fi

FAILED=0

# 使用 null 分隔符处理特殊字符文件名
while IFS= read -r -d '' file; do
    # 跳过工作区已删除的文件（但仍检查暂存区内容）
    [ ! -f "$file" ] && [ ! -e "$file" ] && continue

    # 过滤二进制文件
    if ! file --mime-type "$file" 2>/dev/null | grep -q "text/"; then
        continue
    fi

    staged=$(git show ":$file")
    formatted=$(echo "$staged" | "$CLANG_FORMAT" --assume-filename="$file")

    if [ "$staged" != "$formatted" ]; then
        echo "${RED}✕${RESET} clang-format mismatch: $file"
        echo
        # 使用 diff 的彩色输出
        diff -u <(echo "$staged") <(echo "$formatted") \
            | tail -n +3 \
            | head -50 \
            | sed "s/^-/${RED}-/;s/^+/${GREEN}+/;s/^@/${CYAN}@/" \
            || true
        echo "${RESET}"
        FAILED=1
    fi
done < <(git diff --cached --name-only --diff-filter=ACM -z \
    | grep -zE '\.(c|cc|cpp|cxx|h|hpp)$' \
    | grep -zv '^3dparty/' || true)

if [ "$FAILED" -ne 0 ]; then
    echo "${RED}✋${RESET} clang-format check failed."
    echo "=> 请运行: clang-format -i <files> 并重新 git add"
    exit 1
fi
