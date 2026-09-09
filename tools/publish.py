"""Push the committed release to an explicitly chosen NEW branch, never force.

Uses the operator's existing Git authentication. It does not create repositories,
handle access tokens, or assume authorization to overwrite an existing branch.
"""
from pathlib import Path
import argparse
import re
import subprocess


def git(*args, cwd, capture=False):
    return subprocess.run(['git',*args],cwd=cwd,check=True,text=True,
                          stdout=subprocess.PIPE if capture else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',required=True,help='Exact OWNER/REPOSITORY')
    parser.add_argument('--branch',required=True,help='New remote branch name')
    args=parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+',args.repo):
        parser.error('Use an exact OWNER/REPOSITORY, not a URL or guessed destination')
    root=Path(__file__).resolve().parents[1]
    git('check-ref-format','--branch',args.branch,cwd=root,capture=True)
    status=git('status','--porcelain',cwd=root,capture=True).stdout.strip()
    if status:
        raise SystemExit('Working tree is not clean. Review and commit the intended release before publishing.')
    sha=git('rev-parse','HEAD',cwd=root,capture=True).stdout.strip()
    remote='https://github.com/'+args.repo+'.git'
    branches=git('ls-remote','--heads',remote,cwd=root,capture=True).stdout
    if any(line.endswith('\trefs/heads/'+args.branch) for line in branches.splitlines()):
        raise SystemExit('That remote branch already exists; refusing to update it. Choose a new branch or review the existing history.')
    git('push',remote,sha+':refs/heads/'+args.branch,cwd=root)
    received=git('ls-remote',remote,'refs/heads/'+args.branch,cwd=root,capture=True).stdout.split()[0]
    if received!=sha:
        raise SystemExit('Remote verification failed: branch does not resolve to the expected local commit')
    print('Verified remote commit:',sha)
    print('Repository:',args.repo,'Branch:',args.branch)

if __name__=='__main__':main()
