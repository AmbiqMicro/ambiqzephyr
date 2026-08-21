#!/usr/bin/env python3
# Copyright (c) 2026 Ambiq Micro Inc.
# SPDX-License-Identifier: Apache-2.0
"""Size a twister job timeout from the planned test set and prior run timings.

Reads the testplan for the group being dispatched, looks up how long each
planned instance took in the most recent successful run, and scales the sum by
that run's measured build concurrency.
"""

import argparse
import io
import json
import math
import os
import re
import sys
import urllib.error
import urllib.request
import zipfile
from datetime import datetime

API = "https://api.github.com"
JOB_RE = re.compile(r"twister-build \((?P<group>\S+) (?P<subset>\d+)(?:/\d+)?\)")

OVERHEAD_MIN = 20
MARGIN = 1.5
FLOOR_MIN = 45
CEILING_MIN = 500
FALLBACK_MIN = 180
RUNS = 10
COVERAGE = 0.5
DEFAULT_JOBS = 12.0


class StripAuthRedirect(urllib.request.HTTPRedirectHandler):
    """Artifact downloads redirect to blob storage, which rejects our token."""

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        new = super().redirect_request(req, fp, code, msg, headers, newurl)
        if new is not None:
            for header in list(new.headers):
                if header.lower() == "authorization":
                    del new.headers[header]
        return new


OPENER = urllib.request.build_opener(StripAuthRedirect)


def api(url, token, raw=False):
    req = urllib.request.Request(url, headers={
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json",
    })
    with OPENER.open(req, timeout=60) as resp:
        return resp.read() if raw else json.load(resp)


def seconds_between(start, end):
    fmt = "%Y-%m-%dT%H:%M:%SZ"
    return (datetime.strptime(end, fmt) - datetime.strptime(start, fmt)).total_seconds()


def instance_seconds(entry):
    return max(float(entry.get("build_time") or 0),
               float(entry.get("execution_time") or 0))


def key(platform, name):
    return f"{platform}\t{name}"


def harvest(repo, group, token, workflow, wanted):
    """Merge timings from recent successful runs until the plan is well covered.

    A single run is not enough: an incremental PR run publishes timings for only
    the handful of instances it rebuilt, which can miss a full plan entirely.
    """
    runs = api(f"{API}/repos/{repo}/actions/workflows/{workflow}/runs"
               f"?status=success&per_page={RUNS}", token).get("workflow_runs", [])
    merged, best_serial, best_ratio = {}, 0.0, None
    for run in runs:
        try:
            arts = api(f"{API}/repos/{repo}/actions/runs/{run['id']}/artifacts"
                       f"?per_page=100", token).get("artifacts", [])
            jobs = api(f"{API}/repos/{repo}/actions/runs/{run['id']}/jobs"
                       f"?per_page=100", token).get("jobs", [])
        except urllib.error.URLError:
            continue

        wall = {}
        for job in jobs:
            m = JOB_RE.match(job.get("name", ""))
            if m and m.group("group") == group and job.get("conclusion") == "success":
                if job.get("started_at") and job.get("completed_at"):
                    wall[m.group("subset")] = seconds_between(job["started_at"],
                                                              job["completed_at"])
        if not wall:
            continue

        timings, serial = {}, 0.0
        for art in arts:
            if group not in art.get("name", "") or art.get("expired"):
                continue
            try:
                blob = api(art["archive_download_url"], token, raw=True)
                with zipfile.ZipFile(io.BytesIO(blob)) as zf:
                    member = next((n for n in zf.namelist()
                                   if n.endswith("twister.json")), None)
                    if not member:
                        continue
                    data = json.loads(zf.read(member))
            except (urllib.error.URLError, zipfile.BadZipFile, ValueError, KeyError):
                continue
            for entry in data.get("testsuites", []):
                secs = instance_seconds(entry)
                if secs <= 0:
                    continue
                timings[key(entry.get("platform", ""), entry.get("name", ""))] = secs
                serial += secs

        if not (timings and serial > 0):
            continue
        # Take the ratio from the run with the most build work: on a small
        # incremental run the wall time is nearly all fixed setup, which reads
        # back as no concurrency at all.
        if serial > best_serial:
            best_serial, best_ratio = serial, serial / sum(wall.values())
        for k, v in timings.items():
            merged.setdefault(k, v)
        if len(wanted & merged.keys()) >= COVERAGE * len(wanted):
            break
    if not merged:
        return None, None
    return merged, max(1.0, best_ratio if best_ratio and best_ratio > 1 else DEFAULT_JOBS)


def local_timings(paths):
    """Read timings from twister.json files already on disk."""
    timings = {}
    for path in paths:
        try:
            with open(path) as fh:
                data = json.load(fh)
        except (OSError, ValueError) as err:
            print(f"twister_timeout: {path} unusable ({err})", file=sys.stderr)
            continue
        for entry in data.get("testsuites", []):
            secs = instance_seconds(entry)
            if secs > 0:
                timings[key(entry.get("platform", ""), entry.get("name", ""))] = secs
    return timings


def planned(testplan, group_platforms):
    out = []
    with open(testplan) as fh:
        data = json.load(fh)
    for entry in data.get("testsuites", []):
        platform = entry.get("platform", "")
        if group_platforms and platform not in group_platforms:
            continue
        out.append(key(platform, entry.get("name", "")))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--testplan", default="testplan.json")
    ap.add_argument("--group", required=True)
    ap.add_argument("--nodes", type=int, default=1)
    ap.add_argument("--platforms", default="")
    ap.add_argument("--timings", nargs="+",
                    help="local twister.json files to use instead of the API")
    ap.add_argument("--concurrency", type=float, default=12.0,
                    help="build concurrency assumed with --timings")
    ap.add_argument("--overhead", type=float, default=OVERHEAD_MIN,
                    help="fixed minutes for checkout and west update")
    ap.add_argument("--raw", action="store_true",
                    help="print the unclamped estimate, for comparing against a real run")
    args = ap.parse_args()

    token = os.environ.get("GITHUB_TOKEN", "")
    repo = os.environ.get("GITHUB_REPOSITORY", "")
    workflow = os.environ.get("TWISTER_WORKFLOW_ID", "")

    try:
        instances = planned(args.testplan, set(args.platforms.split()))
    except (OSError, ValueError) as err:
        print(f"twister_timeout: no testplan ({err})", file=sys.stderr)
        print(FALLBACK_MIN)
        return 0

    if not instances:
        print(f"twister_timeout: no instances matched platforms "
              f"{args.platforms!r} in {args.testplan}", file=sys.stderr)
        print(FALLBACK_MIN)
        return 0

    if args.timings:
        timings = local_timings(args.timings)
        concurrency = max(1.0, args.concurrency)
    elif token and repo and workflow:
        try:
            timings, concurrency = harvest(repo, args.group, token, workflow,
                                           set(instances))
        except (urllib.error.URLError, ValueError, KeyError) as err:
            print(f"twister_timeout: history unavailable ({err})", file=sys.stderr)
            timings = None
    else:
        print("twister_timeout: no timing source; pass --timings or set "
              "GITHUB_TOKEN, GITHUB_REPOSITORY and TWISTER_WORKFLOW_ID",
              file=sys.stderr)
        timings = None

    if not timings:
        print("twister_timeout: no usable timings found", file=sys.stderr)
        print(FALLBACK_MIN)
        return 0

    known = [timings[i] for i in instances if i in timings]
    if len(known) < max(10, len(instances) // 10):
        print(f"twister_timeout: only {len(known)} of {len(instances)} planned "
              f"instances have timings", file=sys.stderr)
        print(FALLBACK_MIN)
        return 0

    print(f"twister_timeout: {len(instances)} instances, {len(known)} timed, "
          f"concurrency {concurrency:.1f}, {args.nodes} node(s)", file=sys.stderr)

    default = sum(known) / len(known)
    serial = sum(timings.get(i, default) for i in instances)
    scaled = serial / concurrency / max(1, args.nodes) / 60
    minutes = args.overhead + scaled * MARGIN

    if args.raw:
        print(f"twister_timeout: predicted build {scaled:.1f} min, "
              f"with margin {scaled * MARGIN:.1f} min, "
              f"plus {args.overhead:.0f} min overhead", file=sys.stderr)
        print(round(minutes, 1))
        return 0

    print(int(max(FLOOR_MIN, min(CEILING_MIN, math.ceil(minutes)))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
