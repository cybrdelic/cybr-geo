"""Publish an extracted full workspace with ordinary local Git credentials.

Default is a plan only. --push clones the existing repository, verifies the
full delivery, copies files, commits, and fast-forward pushes. No force push,
credential scraping, secret upload, or repository creation is performed.
"""
from pathlib import Path
import argparse,shutil,subprocess,tempfile,re
from audit_delivery import ROOT,audit
EXCLUDE={'.git','.venv','__pycache__','.pytest_cache','.cache','build','dist','.publication','.publication2'}
def files():
 return [p for p in ROOT.rglob('*') if p.is_file() and not any(s in EXCLUDE for s in p.relative_to(ROOT).parts) and p.suffix not in {'.pyc','.nbc','.nbi','.ttf','.otf'}]
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--repository',default='cybrdelic/cybr-geo');p.add_argument('--branch',default='master');p.add_argument('--push',action='store_true');p.add_argument('--message',default='Publish complete verified geometry workshop and media');a=p.parse_args()
 if not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+',a.repository):p.error('Use owner/repository')
 if a.branch.startswith('-') or '..' in a.branch:p.error('Invalid branch')
 items=files();print(f'{len(items)} files, {sum(f.stat().st_size for f in items):,} bytes -> {a.repository}:{a.branch}')
 if not a.push:print('Plan only. Add --push after authenticating Git on this machine.');return
 if audit()['errors']:raise RuntimeError('Full delivery is missing or modified; inspect the audit before publication')
 with tempfile.TemporaryDirectory(prefix='cybrgeo-publish-') as td:
  dst=Path(td)/'repo';subprocess.run(['git','clone','--branch',a.branch,'--single-branch',f'https://github.com/{a.repository}.git',str(dst)],check=True)
  for src in items:
   rel=src.relative_to(ROOT)
   if rel==Path('LICENSE') and (dst/rel).exists():continue # Preserve existing repository license bytes.
   target=dst/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,target)
  subprocess.run(['git','add','--all'],cwd=dst,check=True)
  if subprocess.run(['git','diff','--cached','--quiet'],cwd=dst).returncode==0:print('No differences');return
  subprocess.run(['git','commit','-m',a.message],cwd=dst,check=True)
  subprocess.run(['git','push','origin',f'HEAD:{a.branch}'],cwd=dst,check=True)
  print(subprocess.check_output(['git','rev-parse','HEAD'],cwd=dst,text=True).strip())
if __name__=='__main__':main()
