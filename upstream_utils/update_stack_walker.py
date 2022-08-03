#!/usr/bin/env python3

import os
import shutil
import subprocess

from upstream_utils import (
    setup_upstream_repo,
    comment_out_invalid_includes,
    walk_cwd_and_copy_if,
    am_patches,
    walk_if,
    copy_to,
)


def crlf_to_lf(stackwalker_dir):
    for root, _, files in os.walk(stackwalker_dir):
        if ".git" in root:
            continue

        for fname in files:
            filename = os.path.join(root, fname)
            print(f"Converting CRLF -> LF for {filename}")
            with open(filename, "rb") as f:
                content = f.read()
                content = content.replace(b"\r\n", b"\n")

                with open(filename, "wb") as f:
                    f.write(content)

    subprocess.check_call(["git", "add", "-A"])
    subprocess.check_call(["git", "commit", "-m", "Fix line endings"])


def main():
    root, repo = setup_upstream_repo(
        "https://github.com/JochenKalmbach/StackWalker",
        "42e7a6e056a9e7aca911a7e9e54e2e4f90bc2652",
        shallow=False,
    )
    wpiutil = os.path.join(root, "wpiutil")

    # Run CRLF -> LF before trying any patches
    crlf_to_lf(repo)

    # Apply patches to original git repo
    patch_dir = os.path.join(root, "upstream_utils/stack_walker_patches")
    am_patches(
        repo,
        [
            os.path.join(patch_dir, "0001-Apply-PR-35.patch"),
            os.path.join(patch_dir, "0002-Remove-_M_IX86-checks.patch"),
            os.path.join(patch_dir, "0003-Add-advapi-pragma.patch"),
        ],
        ignore_whitespce=True,
    )

    shutil.copy(
        os.path.join("Main", "StackWalker", "StackWalker.h"),
        os.path.join(wpiutil, "src/main/native/windows/StackWalker.h"),
    )

    shutil.copy(
        os.path.join("Main", "StackWalker", "StackWalker.cpp"),
        os.path.join(wpiutil, "src/main/native/windows/StackWalker.cpp"),
    )


if __name__ == "__main__":
    main()
