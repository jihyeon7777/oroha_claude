"""Create a GitHub release and upload assets with the git-stored credential (never printed).

  python3 setup/release_upload.py data/backup/20261008/notes.md data/backup/20261008/*.tar.gz data/backup/20261008/SHA256SUMS

Uses the github.com credential stored for user jihyeon7777 (`git credential fill`; saved by the
first `git push`). Creates the release tag data-20261008 on main (prerelease) if it is missing,
then uploads every file that is not attached yet. Run it yourself: it publishes data on a
public repository.
"""
import json
import os
import subprocess
import sys
import urllib.error
import urllib.request

REPO = "jihyeon7777/oroha_claude"
TAG = "data-20261008"


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    cred = subprocess.run(["git", "credential", "fill"],
                          input="protocol=https\nhost=github.com\nusername=jihyeon7777\n\n",
                          capture_output=True, text=True,
                          env={**os.environ, "GIT_TERMINAL_PROMPT": "0"}).stdout
    token = dict(line.split("=", 1) for line in cred.splitlines() if "=" in line).get("password")
    if not token:
        print("no stored github.com credential for jihyeon7777 (run `git push` once first)")
        return 1
    hdr = {"Authorization": "Bearer " + token, "Accept": "application/vnd.github+json",
           "X-GitHub-Api-Version": "2022-11-28"}

    def call(method, url, data=None, extra=None):
        req = urllib.request.Request(url, data=data, method=method, headers={**hdr, **(extra or {})})
        try:
            with urllib.request.urlopen(req, timeout=600) as r:
                return r.status, json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b"{}")

    body = open(sys.argv[1], encoding="utf-8").read()
    st, rel = call("GET", f"https://api.github.com/repos/{REPO}/releases/tags/{TAG}")
    if st != 200:
        st, rel = call("POST", f"https://api.github.com/repos/{REPO}/releases",
                       json.dumps({"tag_name": TAG, "target_commitish": "main",
                                   "name": "Data backup 2026-10-08", "body": body,
                                   "draft": False, "prerelease": True}).encode(),
                       {"Content-Type": "application/json"})
        print("create release:", st, rel.get("html_url") or rel.get("message"))
        if st >= 300:
            return 1
    have = {a["name"] for a in rel.get("assets", [])}
    for path in sys.argv[2:]:
        name = os.path.basename(path)
        if name in have:
            print("already attached:", name)
            continue
        with open(path, "rb") as f:
            data = f.read()
        ctype = "application/gzip" if name.endswith(".gz") else "text/plain"
        st, a = call("POST", f"https://uploads.github.com/repos/{REPO}/releases/{rel['id']}/assets?name={name}",
                     data, {"Content-Type": ctype, "Content-Length": str(len(data))})
        print("upload", name, st, a.get("browser_download_url") or a.get("message"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
