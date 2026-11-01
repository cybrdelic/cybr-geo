$ErrorActionPreference = 'Stop'
$studioPython = Join-Path $PSScriptRoot '..\.venv\Scripts\python.exe'
if (!(Test-Path -LiteralPath $studioPython)) { $studioPython = (Get-Command python).Source }
& $studioPython (Join-Path $PSScriptRoot 'codex_studio.py')
