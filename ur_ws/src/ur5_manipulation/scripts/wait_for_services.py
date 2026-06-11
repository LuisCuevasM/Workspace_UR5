#!/usr/bin/env python3

import argparse
import subprocess
import sys
import time


def _parse_services(value):
    return [item.strip() for item in value.split(",") if item.strip()]


def _list_services():
    result = subprocess.run(
        ["ros2", "service", "list"],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        return set(), result.stderr.strip()

    return set(result.stdout.splitlines()), ""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--services", required=True)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--poll-sec", type=float, default=1.0)
    parser.add_argument("--label", default="stack")
    args = parser.parse_args()

    expected = _parse_services(args.services)
    if not expected:
        print("[ur5_manipulation] No services requested; continuing.", flush=True)
        return 0

    deadline = time.monotonic() + args.timeout
    last_missing = None

    while time.monotonic() < deadline:
        available, error = _list_services()
        missing = [service for service in expected if service not in available]

        if not missing:
            print(
                f"[ur5_manipulation] {args.label} ready: {', '.join(expected)}",
                flush=True,
            )
            return 0

        if missing != last_missing:
            print(
                f"[ur5_manipulation] Waiting for {args.label}: "
                f"{', '.join(missing)}",
                flush=True,
            )
            if error:
                print(f"[ur5_manipulation] ros2 service list: {error}", flush=True)
            last_missing = missing

        time.sleep(args.poll_sec)

    print(
        f"[ur5_manipulation] Timeout waiting for {args.label}: "
        f"{', '.join(last_missing or expected)}",
        file=sys.stderr,
        flush=True,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
