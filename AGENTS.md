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

## Vendor Dependency Management

Vendor libraries live as JSON files in `vendordeps/`. `build.gradle` uses
`wpi.java.vendor.java()` which auto-includes every file in that directory — no
`build.gradle` edit is needed when adding or updating a dep.

There is a helper script at `scripts/update_vendor_deps.py` (also registered as
an OpenCode skill: `.opencode/vendor-dep-update/skill.yaml`).

```bash
# Dry-run — check for missing/outdated deps vs the WPILib catalog (no changes)
python3 scripts/update_vendor_deps.py --catalog

# Check local deps only (vs each dep's own jsonUrl)
python3 scripts/update_vendor_deps.py

# Apply updates — download missing/outdated vendordep JSONs into vendordeps/
python3 scripts/update_vendor_deps.py --catalog --apply

# Apply + commit with jj
python3 scripts/update_vendor_deps.py --catalog --apply --commit

# For a different season year (e.g. next season)
python3 scripts/update_vendor_deps.py --year 2027 --catalog
```

After applying updates, verify the build:

```bash
export JAVA_HOME=/home/jon/wpilib/2026/jdk
export PATH="$JAVA_HOME/bin:$PATH"
./gradlew build
```

The WPILib vendor catalog is maintained at
https://github.com/wpilibsuite/vendor-json-repo (season bundles under `YEAR/`).
The script fetches the catalog from that repo and cross-references it against
your local `vendordeps/` to surface missing or outdated libraries.
