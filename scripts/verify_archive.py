"""Verify saved bytes and compare formatted C++ tokens with the original snapshot."""
from pathlib import Path
import hashlib
import json
import subprocess
import tempfile
from source_tokens import tokens

ROOT = Path(__file__).resolve().parents[1]


def adapted_files(day, version):
    """Reproduce only the declared preset patch; never exempt arbitrary token changes."""
    adaptation = version.get('adaptation')
    if not adaptation:
        return {}
    if day != 'day1' or adaptation['id'] != 'course-preset-v1':
        raise SystemExit('Unknown archive adaptation')
    allowed = {'communication/include/fixed_waypoints.h',
               'communication/include/production_page_ja.h', 'communication/src/main.cpp'}
    if set(adaptation['files']) != allowed:
        raise SystemExit('Unexpected day1 preset file scope')
    patch = ROOT / adaptation['patch']
    if hashlib.sha256(patch.read_bytes()).hexdigest() != adaptation['patch_sha256']:
        raise SystemExit('Preset patch hash mismatch')
    with tempfile.TemporaryDirectory() as tmp:
        folder = Path(tmp)
        for rel in sorted(allowed):
            dest = folder / day / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(subprocess.check_output(
                ['git', 'show', f"{adaptation['base_commit']}:{day}/{rel}"], cwd=ROOT))
        subprocess.run(['git', '-c', 'core.autocrlf=false', 'apply', '--no-index', '--check', '--whitespace=error', str(patch)],
                       cwd=folder, check=True)
        subprocess.run(['git', '-c', 'core.autocrlf=false', 'apply', '--no-index', '--whitespace=error', str(patch)],
                       cwd=folder, check=True)
        return {rel: (folder / day / rel).read_bytes() for rel in allowed}


def verify():
    info = json.loads((ROOT / 'archive.json').read_text())
    baseline = info['original_snapshot_commit']
    for day, version in info['versions'].items():
        adapted = adapted_files(day, version)
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
        for rel, original_hash in version['original_sha256'].items():
            original = subprocess.check_output(
                ['git', 'show', f'{baseline}:{day}/{rel}'], cwd=ROOT)
            current = (ROOT / day / rel).read_bytes()
            if hashlib.sha256(original).hexdigest() != original_hash:
                raise SystemExit(f'{day}/{rel}: original baseline hash mismatch')
            if rel in adapted:
                if current != adapted[rel]:
                    raise SystemExit(f'{day}/{rel}: preset differs from declared patch')
                continue
            if Path(rel).suffix in {'.h', '.cpp', '.inc'}:
                if tokens(original.decode()) != tokens(current.decode()):
                    raise SystemExit(f'{day}/{rel}: non-formatting token change')
            elif original != current:
                raise SystemExit(f'{day}/{rel}: non-C++ file changed')
        if set(version['original_sha256']) != set(actual):
            raise SystemExit(f'{day}: original file list changed')
        print(f'{day}: {len(actual)} files verified; {len(adapted)} declared preset files; '
              'all remaining original C++ tokens and other bytes unchanged')
    return info


if __name__ == '__main__':
    verify()
