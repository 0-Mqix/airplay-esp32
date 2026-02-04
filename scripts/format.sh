#!/bin/bash
cd "$(dirname "$0")/.."
find main \( -name '*.c' -o -name '*.h' \) -exec clang-format -i -style=file {} +