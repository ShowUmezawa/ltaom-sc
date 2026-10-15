"""Check actual compiler dependencies for all built frontend translation units."""
from pathlib import Path
import hashlib
import json
import sys

build, eigen, output = map(Path, sys.argv[1:])
expected = str(eigen.resolve()) + '/'
files = list(build.rglob('*.o.d')) or list(build.rglob('depend.make'))
report = {}
for dep in files:
    paths = {s.rstrip('\\') for s in dep.read_text().split()
             if '/Eigen/' in s or '/unsupported/Eigen/' in s}
    if not paths:
        continue
    bad = sorted(s for s in paths if not s.startswith(expected))
    assert not bad, (str(dep), bad)
    report[str(dep.relative_to(build))] = len(paths)
assert any('fastlio_node' in p for p in report), 'Missing node dependencies'
assert any('extra_lib' in p for p in report), 'Missing extra_lib dependencies'
headers = {str(p.relative_to(eigen)): hashlib.sha256(p.read_bytes()).hexdigest()
           for p in eigen.rglob('*') if p.is_file()}
output.write_text(json.dumps({'version': '3.4.0', 'dependencies': report,
                              'header_sha256': headers}, indent=2) + '\n')
print('Frontend Eigen 3.4.0 dependency gate passed')
