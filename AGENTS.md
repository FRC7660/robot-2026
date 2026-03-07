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
use ~/wpilib/2026/jdk as the jdk
