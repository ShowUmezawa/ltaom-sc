"""Fetch exact dependency commits and apply the recorded local patches once."""
from pathlib import Path
import argparse
import json
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--cache', type=Path, required=True)
args = parser.parse_args()
repo = Path(__file__).resolve().parents[1]
args.cache.mkdir(parents=True, exist_ok=True)
for name, item in json.loads((repo/'locks/build_dependencies.json').read_text()).items():
    dest = args.cache/name
    if dest.exists():
        raise SystemExit('Refusing to overwrite existing dependency: '+str(dest))
    subprocess.run(['git', 'clone', '--no-checkout', item['url'], str(dest)], check=True)
    subprocess.run(['git', '-C', str(dest), 'checkout', '--detach', item['commit']], check=True)
    patch = repo/item['patch']
    if patch.stat().st_size:
        subprocess.run(['git', '-C', str(dest), 'apply', '--check', str(patch)], check=True)
        subprocess.run(['git', '-C', str(dest), 'apply', str(patch)], check=True)
    print(name, item['commit'], 'prepared')
