#!/usr/bin/env python3
"""Download the bags listed in scripts/bags.toml from Hugging Face into bags/.

    python3 scripts/fetch_bags.py                 # every set
    python3 scripts/fetch_bags.py kitti           # one set
    python3 scripts/fetch_bags.py kitti --bag 2011_09_26_drive_0005

Each set in bags.toml names a dataset repo, a commit (revision) and the bag
folders to fetch. Files land in bags/<set>/<path>, the folder the dev
containers mount at /ws/bags. Every file is checked against the hash the Hub
reports for that commit; files that already match are skipped.

Run it on the host from the repo root: standard library only (Python 3.11+).
The dev containers don't mount scripts/, but they see the result at
/ws/bags. Public datasets need no login; set HF_TOKEN to raise the Hub's
rate limits.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys
import tomllib
from urllib.parse import quote
import urllib.request

REPO_ROOT = Path(__file__).resolve().parent.parent
MANIFEST = Path(__file__).with_name('bags.toml')
HUB = 'https://huggingface.co'


def load_manifest(path):
    """bags.toml -> {set: {repo, revision, bags}}, checked."""
    with open(path, 'rb') as f:
        sets = tomllib.load(f)
    for name, s in sets.items():
        missing = {'repo', 'revision', 'bags'} - s.keys()
        if missing:
            sys.exit(f'{path}: [{name}] is missing {", ".join(sorted(missing))}')
        if not re.fullmatch(r'[0-9a-f]{40}', s['revision']):
            sys.exit(f'{path}: [{name}] revision must be a full commit sha, '
                     f'not {s["revision"]!r}, so everyone gets the same files')
    return sets


def request(url):
    headers = {'User-Agent': 'ap1-fetch-bags'}
    token = os.environ.get('HF_TOKEN')
    if token:
        headers['Authorization'] = f'Bearer {token}'
    return urllib.request.urlopen(urllib.request.Request(url, headers=headers))


def list_files(repo, revision):
    """Every file in the dataset at revision: [{path, size, oid, lfs?}]."""
    url = f'{HUB}/api/datasets/{repo}/tree/{revision}?recursive=true'
    files = []
    while url:
        with request(url) as r:
            files += [e for e in json.load(r) if e['type'] == 'file']
            match = re.search(r'<([^>]+)>;\s*rel="next"', r.headers.get('Link', ''))
        url = match.group(1) if match else None
    return files


def file_digest(path, entry):
    """Hash of a local file in the form the Hub reports for entry.

    Large (LFS) files are identified by sha256, small ones by their git blob sha1.
    """
    if 'lfs' in entry:
        h = hashlib.sha256()
    else:
        h = hashlib.sha1()
        h.update(f'blob {path.stat().st_size}\0'.encode())
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def expected_digest(entry):
    return entry['lfs']['oid'] if 'lfs' in entry else entry['oid']


def fetch_file(repo, revision, entry, dest):
    if dest.exists() and file_digest(dest, entry) == expected_digest(entry):
        return False
    dest.parent.mkdir(parents=True, exist_ok=True)
    part = dest.with_name(dest.name + '.part')
    url = f'{HUB}/datasets/{repo}/resolve/{revision}/{quote(entry["path"])}'
    print(f'  {entry["path"]} ({entry["size"] / 1e6:.1f} MB)', flush=True)
    with request(url) as r, open(part, 'wb') as f:
        shutil.copyfileobj(r, f, 1 << 20)
    if file_digest(part, entry) != expected_digest(entry):
        part.unlink()
        sys.exit(f'{entry["path"]}: download does not match the Hub hash; try again')
    part.replace(dest)
    return True


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('sets', nargs='*', help='sets from bags.toml (default: all)')
    p.add_argument('--bag', action='append', help='only this bag of the set (repeatable)')
    p.add_argument('--out', type=Path, default=REPO_ROOT / 'bags')
    args = p.parse_args()

    manifest = load_manifest(MANIFEST)
    unknown = set(args.sets) - manifest.keys()
    if unknown:
        sys.exit(f'unknown set {sorted(unknown)}; bags.toml has {sorted(manifest)}')
    for name in args.sets or manifest:
        s = manifest[name]
        bags = args.bag or s['bags']
        missing = set(bags) - set(s['bags'])
        if missing:
            sys.exit(f'{sorted(missing)} not in set {name}; add them to bags.toml first')
        print(f'{name}: {s["repo"]} @ {s["revision"][:12]}', flush=True)
        files = list_files(s['repo'], s['revision'])
        for bag in bags:
            entries = [e for e in files if e['path'] == bag or e['path'].startswith(bag + '/')]
            if not entries:
                sys.exit(f'{bag} is not in {s["repo"]} at {s["revision"]}')
            fetched = sum(fetch_file(s['repo'], s['revision'], e, args.out / name / e['path'])
                          for e in entries)
            print(f'  {bag}: {fetched} downloaded, {len(entries) - fetched} up to date')


if __name__ == '__main__':
    main()
