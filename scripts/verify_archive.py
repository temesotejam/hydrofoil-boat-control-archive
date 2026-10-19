"""Verify saved bytes and compare formatted C++ tokens with the original snapshot."""
from pathlib import Path
import hashlib
import json
import subprocess
import tempfile
from source_tokens import tokens

ROOT = Path(__file__).resolve().parents[1]


def verify():
    info = json.loads((ROOT / 'archive.json').read_text())
    baseline = info['original_snapshot_commit']
    for day, version in info['versions'].items():
        actual = {
            p.relative_to(ROOT / day).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ROOT / day).rglob('*')
            if p.is_file() and not {'.pio', '__pycache__', '.vscode'}.intersection(p.parts)
        }
        if actual != version['sha256']:
            missing = set(version['sha256']) - set(actual)
            extra = set(actual) - set(version['sha256'])
            changed = [p for p in set(actual) & set(version['sha256'])
                       if actual[p] != version['sha256'][p]]
            raise SystemExit(f'{day}: snapshot mismatch: missing={missing}, extra={extra}, changed={changed}')
        if day == 'day1':
            if version['commit'] != '88bed77ee1779f9e76afa91d313e38ace85177a6' or version['source_snapshot_path'] != 'reconstruction/day1':
                raise SystemExit('Unexpected selected day1 source snapshot')
            for rel in actual:
                selected = subprocess.check_output(['git','show',f"{version['commit']}:{version['source_snapshot_path']}/{rel}"],cwd=ROOT)
                if selected != (ROOT/day/rel).read_bytes():
                    raise SystemExit(f'{day}/{rel}: differs from designated source snapshot')
            print(f'{day}: {len(actual)} files match the designated usage version exactly')
            continue
        for rel, original_hash in version['original_sha256'].items():
            original = subprocess.check_output(
                ['git', 'show', f'{baseline}:{day}/{rel}'], cwd=ROOT)
            current = (ROOT / day / rel).read_bytes()
            if hashlib.sha256(original).hexdigest() != original_hash:
                raise SystemExit(f'{day}/{rel}: original baseline hash mismatch')
            if Path(rel).suffix in {'.h', '.cpp', '.inc'}:
                if tokens(original.decode()) != tokens(current.decode()):
                    raise SystemExit(f'{day}/{rel}: non-formatting token change')
            elif original != current:
                raise SystemExit(f'{day}/{rel}: non-C++ file changed')
        if set(version['original_sha256']) != set(actual):
            raise SystemExit(f'{day}: original file list changed')
        print(f'{day}: {len(actual)} files verified; '
              'all remaining original C++ tokens and other bytes unchanged')
    return info


if __name__ == '__main__':
    verify()
