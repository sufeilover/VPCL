"""Verify packaged inputs without importing simulator code."""
import ast
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def main():
    manifest=json.loads((ROOT/'docs/file_manifest.json').read_text(encoding='utf-8'))
    errors=[]
    for item in manifest['files']:
        path=ROOT/item['path']
        if not path.is_file():errors.append(f'Missing: {item["path"]}');continue
        if hashlib.sha256(path.read_bytes()).hexdigest()!=item['sha256']:errors.append(f'Changed: {item["path"]}')
    for path in ROOT.rglob('*.py'):
        if any(x in path.parts for x in ['outputs','.venv','.git']):continue
        try:ast.parse(path.read_text(encoding='utf-8-sig'),filename=str(path))
        except SyntaxError as exc:errors.append(str(exc))
    if errors:raise SystemExit('\n'.join(errors))
    print(f'Verified {len(manifest["files"])} packaged inputs and Python syntax. Simulator runtime was not executed.')

if __name__=='__main__':main()
