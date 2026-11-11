#!/usr/bin/env python3
"""Snap the trip's GPX tracks to the road network.

Runs on GitHub Actions whenever a GPX file is uploaded. For every GPX file in the
repository (top level and the "gpx" folder) it writes snapped/<file name>.json with a
road-following version of the track. The map page uses that file when it exists and
falls back to the raw GPX otherwise.

How it works: the OSRM map-matching service (OpenStreetMap road data, car profile)
moves the recorded points onto the most plausible roads. Where the recording has a
gap, the gap is filled with a route between the two ends if that route is about as
long as the straight line. Points that are not near any road (desert pistes, ferries)
keep their recorded position.

Only the Python standard library is used. The public OSRM demo server is shared:
the script sends at most one request per second and identifies itself.
"""
from collections import Counter
import hashlib, json, math, os, sys, time, urllib.parse, urllib.request
import xml.etree.ElementTree as ET

VERSION = 2                       # raise to force all tracks to be processed again
SERVERS = [s for s in [os.environ.get("OSRM_URL"),
                       "https://router.project-osrm.org",
                       "https://routing.openstreetmap.de/routed-car"] if s]
REPO = os.environ.get("GITHUB_REPOSITORY", "aljosahafner/transafrica2026-map")
USER_AGENT = f"transafrica2026-map track snapping (https://github.com/{REPO})"
SEARCH_RADIUS_M = 35              # how far a point may be from a road to be snapped
CHUNK = 100                       # OSRM accepts at most 100 points per match request
OUT_DIR = "snapped"

_last_request = 0.0

def get_json(path_and_query):
    """GET from the first OSRM server that answers. Returns parsed JSON."""
    global _last_request
    last_error = None
    for base in SERVERS:
        wait = 1.1 - (time.time() - _last_request)
        if wait > 0:
            time.sleep(wait)
        _last_request = time.time()
        req = urllib.request.Request(base + path_and_query, headers={"User-Agent": USER_AGENT})
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                return json.load(r)
        except urllib.error.HTTPError as e:
            try:
                body = json.load(e)          # OSRM answers "NoMatch" etc. with HTTP 400 and a JSON body
                if "code" in body:
                    return body
            except Exception:
                pass
            last_error = e
        except Exception as e:
            last_error = e
    raise RuntimeError(f"no OSRM server answered: {last_error}")

# ---------- geometry helpers ----------
def dist(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * 6371000 * math.asin(min(1, math.sqrt(h)))

def length(line):
    return sum(dist(a, b) for a, b in zip(line, line[1:]))

def simplify(line, tol_m=5.0):
    """Douglas-Peucker in a local flat projection; keeps the file small."""
    if len(line) < 3:
        return line
    k = math.cos(math.radians(line[0][0]))
    xy = [(p[1] * 111320 * k, p[0] * 110540) for p in line]
    keep = [False] * len(line)
    keep[0] = keep[-1] = True
    stack = [(0, len(line) - 1)]
    while stack:
        i, j = stack.pop()
        (x1, y1), (x2, y2) = xy[i], xy[j]
        dx, dy = x2 - x1, y2 - y1
        norm = math.hypot(dx, dy)
        best, idx = 0.0, None
        for m in range(i + 1, j):
            x, y = xy[m]
            d = abs(dy * (x - x1) - dx * (y - y1)) / norm if norm else math.hypot(x - x1, y - y1)
            if d > best:
                best, idx = d, m
        if idx is not None and best > tol_m:
            keep[idx] = True
            stack += [(i, idx), (idx, j)]
    return [p for p, kp in zip(line, keep) if kp]

# ---------- GPX ----------
def local(tag):
    return tag.rsplit("}", 1)[-1]

def parse_gpx(path):
    root = ET.parse(path).getroot()
    segments, name, meta_name = [], None, None
    for el in root.iter():
        t = local(el.tag)
        if t in ("trkseg", "rte"):
            pts = []
            for p in el:
                if local(p.tag) in ("trkpt", "rtept"):
                    try:
                        lat, lon = float(p.get("lat")), float(p.get("lon"))
                    except (TypeError, ValueError):
                        continue
                    when = None
                    for c in p:
                        if local(c.tag) == "time" and c.text:
                            when = c.text.strip()
                    pts.append((lat, lon, when))
            if len(pts) > 1:
                segments.append(pts)
        elif t == "trk" and name is None:
            for c in el:
                if local(c.tag) == "name" and c.text:
                    name = c.text.strip()
        elif t == "metadata":
            for c in el:
                if local(c.tag) == "name" and c.text:
                    meta_name = c.text.strip()
    times = [p[2] for s in segments for p in s if p[2]]
    title = (name or meta_name or os.path.basename(path))
    if title.lower().endswith(".gpx"):
        title = title[:-4]
    return title, segments, (times[0] if times else None), (times[-1] if times else None)

def to_unix(iso):
    from datetime import datetime
    return int(datetime.fromisoformat(iso.replace("Z", "+00:00")).timestamp())

# ---------- OSRM ----------
def coord_str(points):
    return ";".join(f"{p[1]:.6f},{p[0]:.6f}" for p in points)

STATS = Counter()                 # what OSRM answered, printed in the workflow log
MESSAGES = {}

def note(res):
    code = res.get("code", "?")
    STATS[code] += 1
    if code not in ("Ok", "NoMatch") and res.get("message"):
        MESSAGES.setdefault(code, res["message"])
    return code

def match_request(pts, use_radius, use_time):
    q = {"overview": "full", "geometries": "geojson", "gaps": "ignore"}
    if use_radius:
        q["radiuses"] = ";".join([str(SEARCH_RADIUS_M)] * len(pts))
    if use_time:
        try:
            stamps = [to_unix(p[2]) for p in pts]
            if all(b > a for a, b in zip(stamps, stamps[1:])):
                q["timestamps"] = ";".join(map(str, stamps))
        except Exception:
            pass
    return get_json(f"/match/v1/driving/{coord_str(pts)}?{urllib.parse.urlencode(q, safe=';,')}")

def match_chunk(pts):
    """Returns a list of (first_index, last_index, [[lat, lon], ...]) for the matched stretches."""
    res = None
    # If the server rejects an option, try again with fewer options.
    for use_radius, use_time in ((True, True), (True, False), (False, False)):
        res = match_request(pts, use_radius, use_time)
        code = note(res)
        if code not in ("InvalidOptions", "InvalidValue", "InvalidQuery"):
            break
    if code == "TooBig" and len(pts) > 10:
        half = len(pts) // 2               # too many points for this server: split and retry
        first = match_chunk(pts[:half + 1])
        second = [(lo + half, hi + half, g) for lo, hi, g in match_chunk(pts[half:])]
        return first + second
    if code != "Ok":
        return []
    found = {}
    for i, tp in enumerate(res.get("tracepoints") or []):
        if tp is not None:
            m = tp["matchings_index"]
            lo, hi = found.get(m, (i, i))
            found[m] = (min(lo, i), max(hi, i))
    out = []
    for m, (lo, hi) in found.items():
        mt = res["matchings"][m]
        geom = [[c[1], c[0]] for c in mt["geometry"]["coordinates"]]
        raw_len = length(pts[lo:hi + 1])
        # Reject matches that take an implausible detour compared with what was recorded.
        if hi > lo and len(geom) > 1 and mt.get("distance", 0) <= 1.5 * raw_len + 1000:
            out.append((lo, hi, geom))
        else:
            STATS["rejected detour"] += 1
    return out

def route_between(a, b):
    """A road route between two points, or None if it would be a big detour."""
    straight = dist(a, b)
    try:
        res = get_json(f"/route/v1/driving/{coord_str([a, b])}?overview=full&geometries=geojson")
    except RuntimeError:
        return None
    if note(res) != "Ok" or not res.get("routes"):
        return None
    r = res["routes"][0]
    geom = [[c[1], c[0]] for c in r["geometry"]["coordinates"]]
    # The route must start and end close to the gap's ends and not be a big detour.
    if dist(geom[0], a) > 300 or dist(geom[-1], b) > 300:
        return None
    if r["distance"] > 1.3 * straight + 1500:
        return None
    return geom

def snap_segment(pts):
    """Returns (polyline, share of recorded points that ended up on roads)."""
    n = len(pts)
    matches = []
    start = 0
    while start < n - 1:
        end = min(start + CHUNK, n)
        for lo, hi, geom in match_chunk(pts[start:end]):
            matches.append((start + lo, start + hi, geom))
        start = end - 1                      # chunks share one point so they join up
    matches.sort(key=lambda m: m[0])

    raw = [[p[0], p[1]] for p in pts]
    if not matches:
        return raw, 0.0

    line, pos = [], 0

    def fill_gap(frm, mids, to):
        """Connect frm to to. Recorded points in between are kept as they are."""
        if mids:
            line.extend(mids)
            line.append(to)
        elif dist(frm, to) > 300:
            line.extend((route_between(frm, to) or [frm, to])[1:])
        else:
            line.append(to)

    for lo, hi, geom in matches:
        if hi <= pos and line:
            continue                         # already covered (chunk overlap)
        if not line:
            line.extend(raw[:lo])            # unmatched start of the track
            if line:
                fill_gap(line[-1], [], geom[0])
                line.pop()                   # geom[0] is added just below
            line.extend(geom)
        else:
            mids = raw[pos + 1:lo] if lo > pos else []
            fill_gap(line[-1], mids, geom[0])
            line.extend(geom[1:])
        pos = max(pos, hi)
    if pos < n - 1:                          # unmatched end of the track
        fill_gap(line[-1], raw[pos + 1:-1], raw[-1])
    covered = set()
    for lo, hi, _ in matches:
        covered.update(range(lo, hi + 1))
    return line, len(covered) / n

# ---------- files ----------
def git_blob_sha(path):
    data = open(path, "rb").read()
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()

def find_gpx():
    files = [f for f in os.listdir(".") if f.lower().endswith(".gpx")]
    if os.path.isdir("gpx"):
        files += [os.path.join("gpx", f) for f in os.listdir("gpx") if f.lower().endswith(".gpx")]
    return sorted(files)

def out_path(gpx):
    return os.path.join(OUT_DIR, os.path.basename(gpx) + ".json")

def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    gpx_files = find_gpx()
    wanted = {out_path(g) for g in gpx_files}
    for f in os.listdir(OUT_DIR):                       # remove results of deleted tracks
        p = os.path.join(OUT_DIR, f)
        if p not in wanted:
            os.remove(p)
            print(f"removed {p}")
    failures = 0
    for g in gpx_files:
        target, sha = out_path(g), git_blob_sha(g)
        if os.path.exists(target):
            try:
                old = json.load(open(target))
                if old.get("source_sha") == sha and old.get("version") == VERSION:
                    continue
            except Exception:
                pass
        STATS.clear(); MESSAGES.clear()
        try:
            name, segments, start, end = parse_gpx(g)
            lines, recorded, shares = [], 0, []
            for seg in segments:
                line, share = snap_segment(seg)
                lines.append([[round(p[0], 5), round(p[1], 5)] for p in simplify(line)])
                shares.append(share * len(seg))
                recorded += len(seg)
            km = sum(length(l) for l in lines) / 1000
            share = sum(shares) / recorded if recorded else 0
            answers = ", ".join(f"{k}: {v}" for k, v in STATS.items())
            errors = {k for k in STATS if k not in ("Ok", "NoMatch", "rejected detour")}
            if share == 0 and errors:
                # Nothing matched because the server refused the requests: keep the raw track and retry later.
                details = "; ".join(f"{k}: {m}" for k, m in MESSAGES.items())
                raise RuntimeError(f"OSRM answered {answers}. {details}")
            json.dump({"version": VERSION, "source": os.path.basename(g), "source_sha": sha,
                       "name": name, "start": start, "end": end, "km": round(km, 1),
                       "on_roads": round(share, 3), "segments": lines},
                      open(target, "w"), separators=(",", ":"))
            print(f"{g}: {km:.0f} km, {share:.0%} of points on roads (OSRM answers: {answers})")
        except Exception as e:
            failures += 1
            if os.path.exists(target):
                os.remove(target)              # an outdated result must not hide the problem
            print(f"::warning::{g} could not be fitted to roads, the map shows the recorded track. {e}")
    if failures:
        print(f"{failures} file(s) not fitted; they are tried again on the next upload "
              f"or when the workflow is run by hand.")

if __name__ == "__main__":
    main()
