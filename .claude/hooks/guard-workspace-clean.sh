#!/bin/bash
# Hook: PreToolUse guard for destructive workspace clean operations
# Blocks rm -rf targeting build/, install/, or log/ directories
# unless the user has explicitly asked for it.

INPUT=$(cat)
COMMAND=$(echo "$INPUT" | jq -r '.tool_input.command // empty')

if [ -z "$COMMAND" ]; then
  exit 0
fi

# Check for rm -rf targeting workspace build artifacts
if echo "$COMMAND" | grep -qE 'rm\s+(-[a-zA-Z]*r[a-zA-Z]*f|(-[a-zA-Z]*f[a-zA-Z]*r))\s+.*(build|install|log)'; then
  jq -n '{
    hookSpecificOutput: {
      hookEventName: "PreToolUse",
      permissionDecision: "deny",
      permissionDecisionReason: "Blocked: rm -rf on build/, install/ or log/. Rebuild with /build. If a package needs a clean (for example after switching --symlink-install), ask the user to confirm and let them run the delete, or use `git clean` on untracked paths."
    }
  }'
else
  exit 0
fi
