#!/bin/bash

mkdir -p ~/.config/helix

cat > ~/.config/helix/config.toml << 'EOF'
theme = "meliora"

[editor.cursor-shape]
insert = "block"
normal = "block"
select = "underline"

[editor]
completion-timeout = 100
completion-trigger-len = 2

[editor.lsp]
display-messages = true
display-inlay-hints = true

[keys.insert]
"A-s" = "completion"

EOF


cat > ~/.config/helix/languages.toml << 'EOF'
[[language]]
name = "cpp"
language-servers = [ "clangd" ]
auto-format = true
formatter = { command = "clang-format", args = ["--style=file"] }

[[language]]
name = "c"
language-servers = [ "clangd" ]
auto-format = true
formatter = { command = "clang-format", args = ["--style=file"] }

[language-server.rust-lsp]
command = "rust-analyzer"

[language-server.clangd]
command = "clangd"
args = [
  "--background-index",
  "--clang-tidy",
  "--completion-style=detailed",
  "--header-insertion=iwyu",
  "--pch-storage=memory"
]

[[language]]
name = "rust"
auto-format = true
language-servers = ["rust-analyzer"]
formatter = { command = "cargo", args = ["fmt", "--all", "--"]}
EOF

echo "alias vim=hx" >> ~/.bashrc

hx
