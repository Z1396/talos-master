#!/usr/bin/env bash
set -euo pipefail

# 检测终端是否支持颜色
if [ -t 1 ]; then
    RED=$(tput setaf 1 2>/dev/null || echo '')
    RESET=$(tput sgr0 2>/dev/null || echo '')
else
    RED=''
    RESET=''
fi

FAILED=0

# 使用 null 分隔符处理特殊字符文件名
while IFS= read -r -d '' file; do
    [ ! -f "$file" ] && continue

    # 过滤二进制文件
    mime_type=$(file --mime-type "$file" 2>/dev/null | cut -d: -f2)
    if [[ ! "$mime_type" =~ text/ ]]; then
        continue
    fi

    matches=$(grep -nE 'spdlog::(log|trace|debug|info|warn|error|critical)\s*\(' "$file" 2>/dev/null || true)

    if [ -n "$matches" ]; then
        while IFS=: read -r line_num content; do
            echo "${RED}✕${RESET} spdlog function call detected: $file:$line_num: $(echo "$content" | xargs)"
        done <<< "$matches"
        FAILED=1
    fi
done < <(git diff --cached --name-only --diff-filter=ACM -z \
    | grep -zE '\.(c|cc|cpp|cxx|h|hpp)$' \
    | grep -zv '^3dparty/' || true)

if [ "$FAILED" -ne 0 ]; then
    echo
    echo "${RED}✋${RESET} 禁止使用 spdlog::XXXX()，请改用 SPDLOG_XXXX() 宏"
    exit 1
fi
