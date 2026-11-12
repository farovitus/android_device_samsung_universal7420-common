#!/bin/bash
# Apply the platform patches this device needs to a Lineage source tree.
#
# Usage, from the tree top after repo sync and before building:
#   device/samsung/universal7420-common/patches/apply.sh [TOP]
#
# Each patch sits under patches/<project path>/ and is applied with git am,
# so it becomes a local commit that repo sync rebases. A patch already in
# the tree is skipped, so running the script again is harmless.
set -euo pipefail

TOP=$(cd "${1:-.}" && pwd)
DIR=$(cd "$(dirname "$0")" && pwd)

find "$DIR" -name '*.patch' -print0 | sort -z | while IFS= read -r -d '' patch; do
	rel=${patch#"$DIR"/}
	proj=$TOP/$(dirname "$rel")
	if [ ! -d "$proj/.git" ] && [ ! -f "$proj/.git" ]; then
		echo "apply.sh: $proj is not a git project" >&2
		exit 1
	fi
	if git -C "$proj" apply --check --reverse "$patch" 2>/dev/null; then
		echo "skip  $rel (already applied)"
		continue
	fi
	echo "apply $rel"
	if ! git -C "$proj" am --3way "$patch"; then
		git -C "$proj" am --abort || true
		echo "apply.sh: $rel does not apply" >&2
		exit 1
	fi
done
