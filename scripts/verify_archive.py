"""Check the archived snapshots against the hashes captured from the original commits."""
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[1]
def verify():
 info=json.loads((ROOT/'archive.json').read_text())
 for day,version in info['versions'].items():
  actual={p.relative_to(ROOT/day).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/day).rglob('*') if p.is_file() and '.pio' not in p.parts and '__pycache__' not in p.parts and '.vscode' not in p.parts}
  if actual!=version['sha256']:
   missing=set(version['sha256'])-set(actual);extra=set(actual)-set(version['sha256']);changed=[p for p in set(actual)&set(version['sha256']) if actual[p]!=version['sha256'][p]]
   raise SystemExit(f'{day}: snapshot mismatch: missing={missing}, extra={extra}, changed={changed}')
  print(f'{day}: {len(actual)} original files verified ({version["commit"]})')
 return info
if __name__=='__main__':verify()
