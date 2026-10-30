# raccoon-setup.ps1 — create a new RaccoonOS robot project from the example repo.
#
#   irm https://raw.githubusercontent.com/htl-stp-ecer/raccoon-example/main/raccoon-setup.ps1 | iex
#
# Windows counterpart of raccoon-setup.sh; see that file for the step-by-step overview.

function Invoke-RaccoonSetup {
    $ErrorActionPreference = 'Stop'
    $RepoUrl = 'https://github.com/htl-stp-ecer/raccoon-example'

    function Info($msg) { Write-Host "==> $msg" -ForegroundColor Cyan }
    function Fail($msg) { Write-Host "error: $msg" -ForegroundColor Red; throw $msg }

    # Native commands do not throw on failure, so check the exit code explicitly.
    function Invoke-Native {
        param([string]$Exe)
        & $Exe @args
        if ($LASTEXITCODE -ne 0) { Fail "'$Exe $($args -join ' ')' failed (exit code $LASTEXITCODE)" }
    }

    function Update-SessionPath {
        $machine = [Environment]::GetEnvironmentVariable('Path', 'Machine')
        $user    = [Environment]::GetEnvironmentVariable('Path', 'User')
        $env:Path = "$machine;$user"
    }

    function Write-FileUtf8 {
        param([string]$Path, [string]$Content)
        # UTF-8 without BOM, so tools reading the YAML/TOML are not confused.
        [IO.File]::WriteAllText((Join-Path (Get-Location) $Path), $Content, (New-Object Text.UTF8Encoding($false)))
    }

    # ── 1. Requirements ───────────────────────────────────────────────────────

    if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
        Info 'Installing git'
        if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
            Fail 'winget not found — please install git from https://git-scm.com and re-run'
        }
        Invoke-Native winget install --id Git.Git -e --source winget --accept-package-agreements --accept-source-agreements
        Update-SessionPath
    }

    if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
        Info 'Installing uv'
        Invoke-RestMethod https://astral.sh/uv/install.ps1 | Invoke-Expression
        Update-SessionPath
        $env:Path = "$env:USERPROFILE\.local\bin;$env:Path"
    }
    if (-not (Get-Command git -ErrorAction SilentlyContinue)) { Fail 'git was installed but is not on PATH — open a new terminal and re-run' }
    if (-not (Get-Command uv  -ErrorAction SilentlyContinue)) { Fail 'uv was installed but is not on PATH — open a new terminal and re-run' }

    # ── 2. Project name and type ──────────────────────────────────────────────

    $name = Read-Host 'Project name'
    if ($name -notmatch '^[A-Za-z][A-Za-z0-9_-]*$') {
        Fail "name must start with a letter and contain only letters, digits, '-' or '_'"
    }

    $type = (Read-Host 'Drivetrain type [differential/mecanum]').Trim().ToLower()
    if ($type -notin @('differential', 'mecanum')) { Fail "type must be 'differential' or 'mecanum'" }

    # ── 3. Clone ──────────────────────────────────────────────────────────────

    $here = (Get-Location).Path
    # Probe for write permission by creating and removing a scratch file.
    $probe = Join-Path $here ".raccoon-setup-probe-$([guid]::NewGuid().ToString('N'))"
    try { New-Item -ItemType File -Path $probe -ErrorAction Stop | Out-Null; Remove-Item $probe }
    catch { Fail "no write permission in $here — run this from a directory you own" }

    if (Test-Path $name) { Fail "'$name' already exists in $here" }

    Info "Cloning the $type example into .\$name"
    Invoke-Native git clone --single-branch -b $type $RepoUrl $name
    Set-Location $name

    # ── 4. Rename the project ─────────────────────────────────────────────────

    $pyName = $name.ToLower()
    Info "Renaming project to $name"
    $pyproject = [IO.File]::ReadAllText((Join-Path (Get-Location) 'pyproject.toml'))
    Write-FileUtf8 'pyproject.toml' ($pyproject -replace '(?m)^name = ".*"', "name = `"$pyName`"")

    # ── 5. Fresh repository and uuid ──────────────────────────────────────────

    Info 'Creating a fresh git repository'
    Remove-Item -Recurse -Force .git
    # The example ignores uv.lock; the new project should commit its lockfile.
    if (Test-Path .gitignore) {
        $ignore = [IO.File]::ReadAllText((Join-Path (Get-Location) '.gitignore'))
        Write-FileUtf8 '.gitignore' ($ignore -replace '(?m)^/?uv\.lock[ \t]*\r?\n', '')
    }
    Invoke-Native git init -q -b main

    $uuid = [guid]::NewGuid().ToString()
    $project = [IO.File]::ReadAllText((Join-Path (Get-Location) 'raccoon.project.yml'))
    $project = $project -replace '(?m)^name: .*', "name: $name"
    $project = $project -replace '(?m)^uuid: .*', "uuid: $uuid"
    Write-FileUtf8 'raccoon.project.yml' $project

    # ── 6. Environment and first commit ───────────────────────────────────────

    Info 'Creating the virtual environment (uv sync)'
    Invoke-Native uv sync

    Info 'Creating the initial commit'
    Invoke-Native git add -A
    # Always author the first commit as raccoon-setup, whatever the local git identity is.
    Invoke-Native git -c user.name=raccoon-setup -c user.email=raccoon-setup@localhost commit -q -m 'Initial Commit (created by raccoon-setup)'

    Info 'Done! Next steps:'
    Write-Host "    cd $name"
    Write-Host '    raccoon connect <PI_ADDRESS>'
    Write-Host '    raccoon run'
}

Invoke-RaccoonSetup
