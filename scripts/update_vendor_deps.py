#!/usr/bin/env python3
"""
Check (and optionally apply) updates to FRC vendor dependency JSON files.

Modes:
  default         Check each vendordep in vendordeps/ against its own jsonUrl.
  --catalog       Cross-reference vendordeps/ against the WPILib vendor catalog
                  (wpilibsuite/vendor-json-repo, YEAR bundle) to find missing
                  or outdated deps.

Usage:
    python3 scripts/update_vendor_deps.py                       # dry-run: check local deps vs jsonUrl
    python3 scripts/update_vendor_deps.py --apply               # download updates for local deps
    python3 scripts/update_vendor_deps.py --catalog             # check against WPILib catalog
    python3 scripts/update_vendor_deps.py --catalog --apply     # also download missing/outdated catalog deps
    python3 scripts/update_vendor_deps.py --year 2027 --catalog # use a different season's catalog
    python3 scripts/update_vendor_deps.py --apply --commit      # apply + jj describe
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError

REPO_ROOT = Path(__file__).resolve().parents[1]
VENDORDEPS_DIR = REPO_ROOT / "vendordeps"

VENDOR_JSON_REPO = "https://raw.githubusercontent.com/wpilibsuite/vendor-json-repo/main"
GITHUB_API = "https://api.github.com/repos/wpilibsuite/vendor-json-repo/contents"


def fetch_json(url: str, timeout: int = 20) -> dict:
    req = Request(url, headers={"User-Agent": "frc-vendor-dep-updater/1.0"})
    with urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def version_key(v: str) -> tuple:
    """Turn '2026.0.5' into (2026, 0, 5) for comparison."""
    return tuple(int(x) for x in str(v).split(".") if x.isdigit())


def list_year_directory(year: str) -> list[dict]:
    """Return the GitHub API listing of the YEAR/ directory in vendor-json-repo."""
    url = f"{GITHUB_API}/{year}?ref=main"
    return fetch_json(url)


def build_catalog(year: str) -> dict[str, dict]:
    """
    Build a catalog of vendor name -> {path, version, jsonUrl, uuid, name}
    from the WPILib vendor-json-repo YEAR bundle.

    Strategy:
      1. Fetch YEAR_metadata.json to get the canonical list of vendors (name -> uuid).
      2. List the YEAR/ directory via GitHub API to find all vendordep JSON files.
      3. For each vendor name, pick the highest-version file matching that vendor.
      4. Fetch that file to extract jsonUrl and confirm version.
    """
    metadata_url = f"{VENDOR_JSON_REPO}/{year}_metadata.json"
    try:
        metadata = fetch_json(metadata_url)
    except (URLError, HTTPError, Exception) as exc:
        print(f"[ERR] could not fetch catalog metadata for {year}: {exc}")
        return {}

    # Map vendor name -> uuid from metadata
    vendor_by_uuid = {}
    for entry in metadata:
        name = entry.get("name")
        uuid = entry.get("uuid")
        if name and uuid:
            vendor_by_uuid[uuid] = name

    # List files in the YEAR/ directory
    try:
        files = list_year_directory(year)
    except (URLError, HTTPError, Exception) as exc:
        print(f"[ERR] could not list catalog directory for {year}: {exc}")
        return {}

    # Group files by vendor name. Filename pattern: NAME-VERSION.json or NAME-something.json
    vendor_files: dict[str, list[tuple[str, str]]] = {}  # name -> [(version_str, download_url)]
    for item in files:
        if not isinstance(item, dict):
            continue
        fname = item.get("name", "")
        if not fname.endswith(".json"):
            continue
        download_url = item.get("download_url")
        if not download_url:
            continue

        # Extract vendor name from filename: strip everything after the last version-like segment
        # Patterns seen: REVLib-2026.0.5.json, yagsl-2026.3.12.json, photonlib-v2026.3.1.json,
        #                libgrapplefrc2026.json, Studica-2026.0.0.json
        base = fname[:-5]  # strip .json
        # Try to split off a trailing version (digits/dots) to get the vendor name
        m = re.match(r"^(.*?)[-_]?v?(\d[\d.]*)$", base)
        if m:
            vendor_name = m.group(1).rstrip("-_")
            version_str = m.group(2)
        else:
            # Fallback: use the whole basename as vendor name, version unknown
            vendor_name = base
            version_str = "0"

        vendor_files.setdefault(vendor_name, []).append((version_str, download_url))

    # For each vendor, pick the highest version and fetch its jsonUrl
    catalog: dict[str, dict] = {}
    for vendor_name, entries in vendor_files.items():
        # Sort by version key descending, pick latest
        entries.sort(key=lambda e: version_key(e[0]), reverse=True)
        latest_ver, latest_url = entries[0]

        try:
            dep_json = fetch_json(latest_url)
        except (URLError, HTTPError, Exception):
            continue

        dep_name = dep_json.get("name", vendor_name)
        dep_ver = dep_json.get("version", latest_ver)
        json_url = dep_json.get("jsonUrl")
        uuid = dep_json.get("uuid", "")

        catalog[dep_name] = {
            "name": dep_name,
            "version": dep_ver,
            "jsonUrl": json_url,
            "uuid": uuid,
            "catalog_file": latest_url,
        }

    return catalog


def check_local_deps(apply: bool = False) -> int:
    """Check each vendordep in vendordeps/ against its own jsonUrl."""
    if not VENDORDEPS_DIR.is_dir():
        print(f"ERROR: vendordeps dir not found: {VENDORDEPS_DIR}")
        return 1

    json_files = sorted(VENDORDEPS_DIR.glob("*.json"))
    if not json_files:
        print("No vendor dep JSON files found.")
        return 0

    updates = 0
    for path in json_files:
        local = json.loads(path.read_text())
        name = local.get("name", path.stem)
        local_ver = local.get("version", "?")
        json_url = local.get("jsonUrl")

        if not json_url:
            print(f"[SKIP] {name}: no jsonUrl in {path.name}")
            continue

        try:
            upstream = fetch_json(json_url)
        except (URLError, HTTPError, Exception) as exc:
            print(f"[ERR]  {name}: could not fetch {json_url}: {exc}")
            continue

        upstream_ver = upstream.get("version", "?")
        current = version_key(local_ver)
        latest = version_key(upstream_ver)

        status = "UP-TO-DATE" if current >= latest else "UPDATE AVAILABLE"
        if current < latest:
            updates += 1

        print(f"[{status}] {name}: {local_ver} -> {upstream_ver}  ({path.name})")

        if apply and current < latest:
            path.write_text(json.dumps(upstream, indent=4) + "\n")
            print(f"       wrote updated {path.name}")

    if updates:
        print(f"\n{updates} update(s) available.")
    else:
        print("\nAll vendor deps are up to date.")
    return 0


def check_catalog(apply: bool = False, year: str = "2026") -> int:
    """Cross-reference local vendordeps/ against the WPILib YEAR vendor catalog."""
    print(f"Fetching WPILib {year} vendor catalog...")
    catalog = build_catalog(year)
    if not catalog:
        print(f"ERROR: could not build vendor catalog for {year}.")
        return 1

    print(f"Catalog has {len(catalog)} known vendors.\n")

    # Load local vendordeps
    local_deps: dict[str, dict] = {}
    if VENDORDEPS_DIR.is_dir():
        for path in sorted(VENDORDEPS_DIR.glob("*.json")):
            try:
                dep = json.loads(path.read_text())
                local_deps[dep.get("name", path.stem)] = {
                    "path": path,
                    "version": dep.get("version", "?"),
                    "jsonUrl": dep.get("jsonUrl"),
                    "uuid": dep.get("uuid", ""),
                }
            except (json.JSONDecodeError, OSError):
                continue

    missing = []
    outdated = []
    up_to_date = []

    for name, cat_entry in sorted(catalog.items()):
        local = local_deps.get(name)
        if local is None:
            missing.append((name, cat_entry))
            continue

        cat_ver = version_key(cat_entry["version"])
        local_ver = version_key(local["version"])

        if local_ver < cat_ver:
            outdated.append((name, local, cat_entry))
        else:
            up_to_date.append((name, local, cat_entry))

    # Report missing
    if missing:
        print("=== MISSING from vendordeps/ (in WPILib catalog) ===")
        for name, entry in missing:
            print(f"  [MISSING] {name} v{entry['version']}  (uuid: {entry['uuid'][:8]}...)")
            if entry["jsonUrl"]:
                print(f"            jsonUrl: {entry['jsonUrl']}")
        print()

    # Report outdated
    if outdated:
        print("=== OUTDATED (catalog has newer version) ===")
        for name, local, cat_entry in outdated:
            print(f"  [OUTDATED] {name}: {local['version']} -> {cat_entry['version']}")
            if cat_entry["jsonUrl"]:
                print(f"             jsonUrl: {cat_entry['jsonUrl']}")
        print()

    # Report up to date
    if up_to_date:
        print("=== UP-TO-DATE ===")
        for name, local, _ in up_to_date:
            print(f"  [OK] {name} v{local['version']}")
        print()

    total_issues = len(missing) + len(outdated)
    print(f"{total_issues} issue(s) found in catalog comparison.")

    if apply:
        installed = 0
        for name, entry in missing:
            if entry["jsonUrl"]:
                try:
                    dep_json = fetch_json(entry["jsonUrl"])
                    # Determine filename from the vendordep JSON's fileName or name+version
                    fname = dep_json.get("fileName") or f"{name}-{entry['version']}.json"
                    out_path = VENDORDEPS_DIR / fname
                    out_path.write_text(json.dumps(dep_json, indent=4) + "\n")
                    print(f"  installed {fname}")
                    installed += 1
                except (URLError, HTTPError, Exception) as exc:
                    print(f"  [ERR] could not install {name}: {exc}")
        for name, local, cat_entry in outdated:
            if cat_entry["jsonUrl"]:
                try:
                    dep_json = fetch_json(cat_entry["jsonUrl"])
                    fname = dep_json.get("fileName") or f"{name}-{cat_entry['version']}.json"
                    out_path = VENDORDEPS_DIR / fname
                    out_path.write_text(json.dumps(dep_json, indent=4) + "\n")
                    print(f"  updated {fname}")
                    installed += 1
                except (URLError, HTTPError, Exception) as exc:
                    print(f"  [ERR] could not update {name}: {exc}")
        print(f"\n{installed} file(s) written to vendordeps/.")

    return 0


def jj_commit(message: str) -> None:
    """Commit changes with jj if we're in a jj repo."""
    try:
        subprocess.run(["jj", "describe", "-m", message], check=True, cwd=REPO_ROOT)
        print(f"Committed with jj: {message}")
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        print(f"WARNING: jj commit failed ({exc}). Commit manually.")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check/update FRC vendor dependency JSONs.")
    parser.add_argument("--catalog", action="store_true",
                        help="Cross-reference against the WPILib vendor catalog.")
    parser.add_argument("--year", default="2026",
                        help="Season year for the WPILib vendor catalog (default: 2026).")
    parser.add_argument("--apply", action="store_true",
                        help="Download updated/missing JSON files.")
    parser.add_argument("--commit", action="store_true",
                        help="jj describe after applying updates.")
    args = parser.parse_args()

    if args.commit and not args.apply:
        print("ERROR: --commit requires --apply.")
        return 1

    if args.catalog:
        rc = check_catalog(apply=args.apply, year=args.year)
    else:
        rc = check_local_deps(apply=args.apply)

    if args.apply and rc == 0:
        jj_commit("chore: update vendor dependencies")

    return rc


if __name__ == "__main__":
    sys.exit(main())
