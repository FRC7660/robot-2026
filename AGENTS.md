# CLAUDE.md

## Robot log workflow

Use SSH to inspect logs on the robot. 

```bash
ssh admin@10.76.60.2
```

Log location:
/var/local/natinst/log/FRC_UserProgram.log

## Build and deploy commands

From the repo root:

```bash
# Compile only
./gradlew compileJava

# Full build and tests
./gradlew build

# Deploy robot code
./gradlew deploy

# Deploy to a specific team number (optional)
./gradlew deploy -PteamNumber=7660
```

## Java check (WPILib JDK)

Run this before compile/deploy:

```bash
JAVA_BIN="${JAVA_HOME:+$JAVA_HOME/bin/java}"
if [ -z "$JAVA_BIN" ] || [ ! -x "$JAVA_BIN" ]; then
  JAVA_BIN="$(command -v java || true)"
fi

JAVA_REAL="$(readlink -f "$JAVA_BIN" 2>/dev/null || echo "$JAVA_BIN")"
JAVA_VER="$($JAVA_BIN -version 2>&1 | head -n 1)"

echo "Using Java: $JAVA_REAL"
echo "Version: $JAVA_VER"

if ! echo "$JAVA_REAL" | grep -qi "wpilib"; then
  echo "WARNING: Active Java does not appear to be the WPILib JDK."
  echo "Expected path to include 'wpilib' (for example ~/wpilib/YYYY/jdk)."
fi
```
