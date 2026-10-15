"""Bind new runs to a commit and the exact tracked working-tree contents."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

def record(repo, output, args):
    def git(*cmd):
        return subprocess.check_output(['git', '--git-dir='+str(repo/'.git'), '--work-tree='+str(repo), *cmd], text=True).strip()
    files = git('ls-files', '-z').split('\0')
    hashes = {n: hashlib.sha256((repo/n).read_bytes()).hexdigest()
              for n in files if n and (repo/n).is_file()}
    result = dict(commit=git('rev-parse', 'HEAD'), branch=git('branch', '--show-current'),
                  dirty=bool(git('status', '--porcelain')), tracked_sha256=hashes,
                  arguments={k:str(v) for k,v in vars(args).items()},
                  image_id=os.getenv('LTAOM_IMAGE_ID', 'not recorded'),
                  parity_capture=os.getenv('LTAOM_PARITY_CAPTURE', '0'))
    (output/'run_provenance.json').write_text(json.dumps(result, indent=2)+'\n')
