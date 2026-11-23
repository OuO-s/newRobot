#!/usr/bin/env bash
# 一键上传到 GitHub (newRobot 仓库 chassis-nav 分支)
# 用法:  ./upload.sh "备注内容"     (备注可省略, 默认=日期)
set -euo pipefail
cd "$(dirname "$0")"

msg="${1:-$(date +%m-%d) 更新}"
git add -A
if git diff --cached --quiet; then
    echo "✅ 没有新改动, 无需上传"
    exit 0
fi
git commit -m "$msg"
echo "📦 本地提交完成: $msg"

# GitHub 通道时通时断, 最多重试 5 次
for i in $(seq 1 5); do
    echo "== 推送第 $i 次 =="
    if err=$(GIT_TERMINAL_PROMPT=0 timeout 45 git push 2>&1); then
        echo "✅ 上传成功"
        git status -sb | head -1
        exit 0
    fi
    if echo "$err" | grep -q "could not read Username"; then
        echo "❌ 本机未存 GitHub 凭据。解决方法见 README 或让 Claude 配置 credential.helper"
        exit 1
    fi
    sleep 5
done
echo "❌ 推送失败(GitHub 通道不通)。本地提交已保留, 稍后重跑 ./upload.sh 即可(会自动跳过已提交内容)"
exit 1
