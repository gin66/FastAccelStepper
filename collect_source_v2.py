#!/usr/bin/env python3
"""
collect_source.py
-----------------
Chạy script này trong thư mục root của project.
Kết quả: [tên thư mục root].txt chứa cây thư mục ASCII + toàn bộ nội dung source code.
"""

import os
import sys

# ── Cấu hình ──────────────────────────────────────────────────────────────────

# Phần mở rộng được coi là source code (thêm/bớt tuỳ ý)
SOURCE_EXTENSIONS = {
    ".py", ".js", ".ts", ".jsx", ".tsx",
    ".java", ".kt", ".scala",
    ".c", ".cpp", ".cc", ".cxx", ".h", ".hpp",
    ".cs", ".go", ".rs", ".swift",
    ".rb", ".php", ".lua", ".r",
    ".html", ".htm", ".css", ".scss", ".sass", ".less",
    ".sh", ".bash", ".zsh", ".fish", ".ps1", ".bat", ".cmd",
    ".sql", ".graphql", ".proto",
    ".json", ".yaml", ".yml", ".toml", ".ini", ".cfg", ".conf", ".env",
    ".xml", ".md", ".rst", ".txt",
    ".dockerfile", ".makefile",
    # thêm tuỳ ý...
}

# Tên file / thư mục cần bỏ qua hoàn toàn
IGNORE_NAMES = {
    ".git", ".svn", ".hg",
    "__pycache__", ".mypy_cache", ".pytest_cache",
    "node_modules", ".npm", ".yarn",
    "venv", ".venv", "env", ".env",
    ".idea", ".vscode",
    "dist", "build", "out", ".next", ".nuxt",
    "coverage", ".coverage",
}

# ── Helpers ────────────────────────────────────────────────────────────────────

def should_ignore(name: str) -> bool:
    return name in IGNORE_NAMES or name.startswith(".")


def is_source_file(name: str) -> bool:
    _, ext = os.path.splitext(name)
    return ext.lower() in SOURCE_EXTENSIONS


def build_tree(root: str, prefix: str = "") -> list[str]:
    """Trả về danh sách dòng ASCII tree."""
    lines = []
    try:
        entries = sorted(os.scandir(root), key=lambda e: (not e.is_dir(), e.name.lower()))
    except PermissionError:
        return lines

    entries = [e for e in entries if not should_ignore(e.name)]

    for i, entry in enumerate(entries):
        connector = "└── " if i == len(entries) - 1 else "├── "
        lines.append(f"{prefix}{connector}{entry.name}")
        if entry.is_dir():
            extension = "    " if i == len(entries) - 1 else "│   "
            lines.extend(build_tree(entry.path, prefix + extension))
    return lines


def collect_files(root: str) -> list[str]:
    """Trả về danh sách đường dẫn tuyệt đối của tất cả source file, theo thứ tự."""
    result = []
    for dirpath, dirnames, filenames in os.walk(root, topdown=True):
        # Lọc thư mục bị ignore (chỉnh sửa in-place để os.walk không đi vào)
        dirnames[:] = sorted(
            [d for d in dirnames if not should_ignore(d)]
        )
        for fname in sorted(filenames):
            if not should_ignore(fname) and is_source_file(fname):
                result.append(os.path.join(dirpath, fname))
    return result


def read_file_safe(path: str) -> str:
    """Đọc file, fallback sang latin-1 nếu UTF-8 lỗi."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read()
    except UnicodeDecodeError:
        with open(path, "r", encoding="latin-1") as f:
            return f.read()


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    root = os.path.abspath(".")
    root_name = os.path.basename(root)
    output_path = os.path.join(root, f"{root_name}.txt")

    print(f"📁 Root   : {root}")
    print(f"📄 Output : {output_path}")

    source_files = collect_files(root)

    # Loại output file và bản thân script khỏi danh sách
    script_path = os.path.abspath(__file__)
    source_files = [
        f for f in source_files
        if os.path.abspath(f) != os.path.abspath(output_path)
        and os.path.abspath(f) != script_path
    ]

    with open(output_path, "w", encoding="utf-8") as out:
        # ── 1. Cây thư mục ──
        out.write(f"{root_name}/\n")
        tree_lines = build_tree(root)
        out.write("\n".join(tree_lines))
        out.write("\n\n")
        out.write("=" * 60 + "\n\n")

        # ── 2. Nội dung từng file ──
        for fpath in source_files:
            rel_path = os.path.relpath(fpath, root)
            fname = os.path.basename(fpath)

            out.write(f"📄 FILE: {rel_path}\n")
            out.write("-" * 60 + "\n")
            out.write(read_file_safe(fpath))
            # Đảm bảo luôn xuống dòng trước footer
            out.write("\n")
            out.write(f"======end of [{fname}]======\n\n")

    print(f"✅ Đã ghi {len(source_files)} file vào {output_path}")


if __name__ == "__main__":
    main()
