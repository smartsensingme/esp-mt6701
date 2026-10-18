#!/bin/sh
set -eu
cd "$(dirname "$0")"
if [ "$(uname -s)" = Darwin ] && [ -z "${SDKROOT:-}" ]; then
    SDKROOT=$(xcrun --sdk macosx --show-sdk-path)
    export SDKROOT
fi
test_binary=$(mktemp "${TMPDIR:-/tmp}/mt6701-recovery.XXXXXX")
trap 'rm -f "$test_binary"' EXIT
${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Imain test_encoder_recovery.c -o "$test_binary"
"$test_binary"
