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
      permissionDecisionReason: "Blocked: destructive workspace clean (rm -rf build/install/log). Use /build to rebuild instead, or explicitly ask the user to confirm workspace clean."
    }
  }'
else
  exit 0
fi
