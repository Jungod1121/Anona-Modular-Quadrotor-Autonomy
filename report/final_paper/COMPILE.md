# 正式报告编译说明

## 依赖

```bash
sudo apt install -y texlive-xetex texlive-lang-chinese texlive-fonts-recommended \
  texlive-latex-extra latexmk fonts-noto-cjk
```

本机需有 `Noto Serif CJK SC` / `Noto Sans CJK SC`。无 Times New Roman 时使用 Liberation Serif（已写入 `main-arxiv.tex`）。

## 编译

```bash
cd report/final_paper
latexmk -xelatex -interaction=nonstopmode -shell-escape main-arxiv.tex
```

产物：`main-arxiv.pdf`。

## 版式约定

- 第 1 页单栏：标题、作者、摘要、关键词
- 第 2 页起单栏：目录
- 其后双栏正文
- 图题「图 N」、表题「表 N」（非「论文 picture」）

写作规范见 `WRITING_SPEC.md`。
