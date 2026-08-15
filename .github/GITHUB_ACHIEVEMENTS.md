# GitHub 协作基础操作指南

> 本文档整理 GitHub 日常协作中需要掌握的基础知识与操作步骤，包括 Issue、Pull Request、Code Review、分支管理、Discussions 等核心机制的实际工作流程。

---

## 一、Issue：任务追踪与讨论

### 1.1 基本概念

Issue 是 GitHub 的任务追踪单元，用于：
- 报告 Bug
- 提出功能需求
- 讨论设计方向
- 记录待办事项

每个 Issue 有唯一编号（如 `#42`），可被引用、关联、关闭。

### 1.2 创建 Issue

1. 进入仓库 `Issues` 标签页
2. 点击 `New issue`
3. 填写标题与正文（支持 Markdown）
4. 选择性地添加：标签（Label）、指派人（Assignee）、里程碑（Milestone）、关联项目（Project）
5. 点击 `Submit new issue`

### 1.3 Issue 正文规范

良好的 Issue 通常包含：

```markdown
## 问题描述
简要说明遇到的问题或需求。

## 复现步骤
1. 打开 ...
2. 点击 ...
3. 出现 ...

## 期望行为
应当 ...

## 实际行为
实际出现了 ...

## 环境信息
- 操作系统：
- 软件版本：
- 相关配置：
```

### 1.4 关闭与重新开启

- **关闭**：问题解决或不再处理后，点击底部 `Close issue`
- **重新开启**：若关闭后发现仍需处理，点击 `Reopen issue`
- **关联提交关闭**：在 commit message 中写 `fixes #42` / `closes #42` / `resolves #42`，合并到默认分支后会自动关闭对应 Issue

### 1.5 引用语法

| 语法 | 含义 |
|------|------|
| `#42` | 引用本仓库的 Issue/PR #42 |
| `user/repo#42` | 引用其它仓库的 Issue/PR |
| `@username` | 提及某人，会收到通知 |
| `commit-sha` | 引用某个提交 |

---

## 二、Pull Request：代码变更的核心协作机制

### 2.1 基本概念

Pull Request（PR）是请求目标仓库拉取（pull）自己分支代码的协作机制，是代码进入主干的标准入口。核心作用：
- 让他人审查代码
- 触发 CI 自动化测试
- 留下变更记录与讨论上下文
- 控制代码合并权限

### 2.2 创建 PR 的完整流程

#### 方式一：在原仓库内协作（直接推送权限）

```bash
# 1. 同步最新主干
git checkout main
git pull origin main

# 2. 创建特性分支（命名建议：feature/xxx, fix/xxx, docs/xxx）
git checkout -b feature/add-armor-filter

# 3. 修改代码并提交（可多次提交）
git add .
git commit -m "feat(armor): add light filter to reduce false positives"

# 4. 推送到远程
git push -u origin feature/add-armor-filter
```

推送后，终端会输出一个创建 PR 的链接，点击即可。

#### 方式二：通过 Fork 协作（无直接推送权限）

```bash
# 1. 在 GitHub 网页上 Fork 目标仓库到自己的账号
# 2. Clone 自己的 Fork 到本地
git clone https://github.com/<你的用户名>/<仓库名>.git
cd <仓库名>

# 3. 配置上游远程，便于同步原仓库更新
git remote add upstream https://github.com/<原作者>/<仓库名>.git

# 4. 同步主干
git checkout main
git fetch upstream
git merge upstream/main
git push origin main

# 5. 创建特性分支并开发
git checkout -b feature/xxx
# 修改代码
git commit -m "..."
git push -u origin feature/xxx

# 6. 在 GitHub 网页上选择 base: <原作者>/<仓库名>:main  ←  head: <你的用户名>/<仓库名>:feature/xxx，创建 PR
```

### 2.3 PR 描述规范

```markdown
## 变更说明
本次 PR 做了什么、为什么。

## 关联 Issue
Closes #42

## 变更类型
- [ ] Bug 修复
- [ ] 新功能
- [ ] 重构
- [ ] 文档
- [ ] 其他

## 测试方式
1. ...
2. ...

## 检查清单
- [ ] 已通过本地编译
- [ ] 已运行单元测试
- [ ] 已更新相关文档
```

### 2.4 Code Review 流程

1. **审查人**收到 PR 通知后进入 PR 页面
2. 在 `Files changed` 标签下逐行查看代码
3. 鼠标悬停在某行可添加行内评论
4. 顶部可对整个 PR 添加整体评论
5. 审查完成后选择：
   - `Comment`：仅评论，不表态
   - `Approve`：通过审查
   - `Request changes`：要求修改后才能合并
6. **作者**根据评论修改代码并继续推送到同一分支，PR 会自动更新
7. 直到所有审查人 Approve 且 CI 通过，方可合并

### 2.5 合并策略

| 策略 | 行为 | 适用场景 |
|------|------|----------|
| `Create a merge commit` | 保留所有提交并新增一个合并提交 | 默认策略，保留完整历史 |
| `Squash and merge` | 将所有提交压缩为一个提交合并 | 保持主干历史整洁 |
| `Rebase and merge` | 将分支提交逐个变基到主干顶端 | 保持线性历史 |

> 选择建议：特性分支用 Squash，长期协作分支用 Merge commit。

### 2.6 冲突解决

当 PR 显示 `This branch has conflicts that must be resolved` 时：

```bash
# 1. 本地同步主干
git checkout feature/xxx
git fetch origin
git merge origin/main    # 或 git rebase origin/main

# 2. 手动编辑冲突文件，解决 <<<<<<< ======= >>>>>>> 标记
# 3. 标记解决
git add .
git commit               # merge 方式
# 或 git rebase --continue  rebase 方式

# 4. 推送
git push origin feature/xxx
```

也可在 GitHub 网页上对简单冲突直接编辑解决。

---

## 三、分支管理

### 3.1 常见分支命名约定

| 前缀 | 用途 | 示例 |
|------|------|------|
| `feature/` | 新功能 | `feature/armor-tracker` |
| `fix/` | Bug 修复 | `fix/memory-leak` |
| `hotfix/` | 紧急修复 | `hotfix/crash-on-start` |
| `docs/` | 文档变更 | `docs/api-usage` |
| `refactor/` | 代码重构 | `refactor/extract-solver` |
| `release/` | 发布分支 | `release/v1.0` |

### 3.2 主干分支模型

- `main` / `master`：稳定可发布版本
- `develop`：开发集成分支（部分团队采用）
- `feature/*`：特性开发分支，源自 `develop`，回归 `develop`
- `release/*`：发布准备分支
- `hotfix/*`：紧急修复分支，源自 `main`

### 3.3 同步与清理

```bash
# 同步远程主干
git fetch origin
git checkout main
git pull --ff-only origin main

# 删除已合并的本地分支
git branch --merged main | grep -v '^\*\|main' | xargs -n 1 git branch -d

# 删除已合并的远程分支
git push origin --delete feature/xxx
```

---

## 四、Commit Message 规范

### 4.1 Conventional Commits 格式

```
<type>(<scope>): <subject>

<body>

<footer>
```

### 4.2 常用 type

| type | 含义 |
|------|------|
| `feat` | 新功能 |
| `fix` | Bug 修复 |
| `docs` | 文档变更 |
| `style` | 代码格式（不影响逻辑） |
| `refactor` | 重构 |
| `perf` | 性能优化 |
| `test` | 测试相关 |
| `build` | 构建系统、依赖变更 |
| `ci` | CI 配置变更 |
| `chore` | 杂项 |

### 4.3 示例

```
feat(armor): add light bar filter to reduce false positives

Add a length-ratio filter for detected light bars, dropping candidates
whose aspect ratio is below 2.0. Reduces false positive rate by ~30%
on test dataset.

Closes #42
```

---

## 五、Discussions：社区问答与长讨论

### 5.1 与 Issue 的区别

| 维度 | Issue | Discussions |
|------|-------|-------------|
| 用途 | 具体任务、可追踪 | 开放式讨论、问答 |
| 状态 | open / closed | 多种分类、可标记答案 |
| 适用 | Bug、明确需求 | 设计讨论、求助、Show & Tell |

### 5.2 开启 Discussions

`Settings → Features → 勾选 Discussions`。

### 5.3 分类管理

默认分类：
- **Announcements**：公告（仅维护者发布）
- **Ideas**：想法建议
- **Q&A**：问答（可标记答案）
- **Show and tell**：项目展示

可在 `Settings → Discussions` 自定义分类。

### 5.4 标记答案

在 Q&A 分类下，回答下方点击 `Mark as answer`，问题状态变为已解决。

---

## 六、Fork、Clone、Remote 的关系

| 概念 | 含义 |
|------|------|
| **Clone** | 把远程仓库完整复制到本地 |
| **Fork** | 在 GitHub 上复制一份仓库到自己账号下 |
| **Remote** | 本地仓库记录的远程地址，可有多个 |

典型配置（Fork 协作模式）：

```bash
git remote -v
# origin    https://github.com/<你>/<仓库>.git  (fetch/push)   —— 你可推送的 Fork
# upstream  https://github.com/<原作者>/<仓库>.git (fetch)      —— 只拉取的原仓库
```

同步 upstream 更新到自己的 Fork：

```bash
git checkout main
git fetch upstream
git merge upstream/main
git push origin main
```

---

## 七、Release 与 Tag

### 7.1 创建 Tag

```bash
# 轻量 tag
git tag v1.0.0

# 附注 tag（推荐，包含作者、日期、说明）
git tag -a v1.0.0 -m "Release version 1.0.0"

# 推送单个 tag
git push origin v1.0.0

# 推送所有 tag
git push origin --tags
```

### 7.2 发布 Release

1. 进入仓库 `Releases` 标签
2. 点击 `Draft a new release`
3. 选择 tag（或新建）
4. 填写标题与发布说明（Changelog）
5. 可上传二进制附件
6. 点击 `Publish release`

### 7.3 语义化版本号

```
MAJOR.MINOR.PATCH
   1     2     3
```

- MAJOR：不兼容的 API 变更
- MINOR：向后兼容的新功能
- PATCH：向后兼容的 Bug 修复

---

## 八、CI/CD 与 PR 的联动

### 8.1 GitHub Actions 基础

工作流文件位于 `.github/workflows/*.yml`，PR 创建/更新时自动触发。

最小示例：

```yaml
name: CI

on:
  pull_request:
    branches: [main]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Build
        run: |
          cmake -B build
          cmake --build build
      - name: Test
        run: ctest --test-dir build
```

### 8.2 PR 状态检查

- PR 页面底部显示所有检查项状态
- `Required` 检查失败会阻止合并（在分支保护规则中配置）
- 点击 `Details` 可查看失败日志

### 8.3 分支保护规则

`Settings → Branches → Add rule`，常用配置：
- Require a pull request before merging
- Require status checks to pass（指定必需的 CI 任务）
- Require conversation resolution before merging
- Require linear history

---

## 九、通知与协作礼仪

### 9.1 通知类型

- `@mention`：被提及
- `Assign`：被指派
- `Review requested`：被请求审查
- `Watch`：关注的仓库有活动

### 9.2 协作建议

- PR 描述清晰，关联 Issue
- 单个 PR 控制在小范围，便于审查
- Commit message 遵循规范，便于追溯
- Review 时对事不对人，给出具体建议
- 收到 Review 意见后及时回复或修改
- 长时间未响应的 PR 主动 ping 一下
- 不要在他人仓库直接 force push 共享分支

---

## 十、常用查询 URL

| 用途 | URL |
|------|-----|
| 我的 PR | `https://github.com/pulls` |
| 分配给我的 Issue | `https://github.com/issues/assigned` |
| 我创建的 Issue | `https://github.com/issues/created` |
| 仓库已合并 PR | `https://github.com/<user>/<repo>/pulls?q=is%3Apr+is%3Amerged` |
| 仓库开放 Issue | `https://github.com/<user>/<repo>/issues?q=is%3Aissue+is%3Aopen` |
| 提及我的内容 | `https://github.com/notifications` |

---

## 附：日常协作速查

```bash
# 开始新任务
git checkout main && git pull
git checkout -b feature/xxx

# 开发中保存进度
git add . && git commit -m "wip: ..."

# 同步主干变更
git fetch origin
git rebase origin/main      # 或 git merge origin/main

# 推送
git push -u origin feature/xxx

# 修改上一个提交
git commit --amend --no-edit
git push --force-with-lease # 安全的强推

# 撤销已 push 的提交（保留历史）
git revert <commit-sha>
git push

# 查看改动
git diff                    # 工作区 vs 暂存区
git diff --staged           # 暂存区 vs HEAD
git log --oneline -10       # 最近 10 个提交
```
