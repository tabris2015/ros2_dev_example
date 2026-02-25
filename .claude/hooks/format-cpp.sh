#!/bin/bash
# Hook: PostToolUse auto-format for C++ files after Edit/Write
# Runs ament_uncrustify --reformat on the edited file if it's a C++ source.

INPUT=$(cat)

# Extract the file path from the tool input
# Edit tool uses file_path, Write tool uses file_path
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

if [ -z "$FILE_PATH" ]; then
  exit 0
fi

# Check if it's a C++ file
case "$FILE_PATH" in
  *.cpp|*.hpp|*.h|*.cc|*.cxx|*.hxx)
    # Only format if ament_uncrustify is available
    if command -v ament_uncrustify &> /dev/null; then
      ament_uncrustify --reformat "$FILE_PATH" 2>/dev/null || true
    fi
    ;;
esac

exit 0
