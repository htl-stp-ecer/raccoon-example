#!/usr/bin/env bash
# raccoon-setup.sh — create a new RaccoonOS robot project from the example repo.
#
#   curl -LsSf https://raw.githubusercontent.com/htl-stp-ecer/raccoon-example/main/raccoon-setup.sh | bash
#
# What it does:
#   1. installs git and uv if they are missing
#   2. asks for a project name and drivetrain type (d = differential, m = mecanum)
#   3. clones the matching branch of the example repo into ./<project name>
#   4. renames the project in pyproject.toml and raccoon.project.yml
#   5. replaces the example's .git with a fresh repository and a fresh project uuid
#   6. runs `uv sync` and makes the initial commit
set -euo pipefail

REPO_URL="https://github.com/htl-stp-ecer/raccoon-example"

info() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# When piped through `curl | bash`, stdin is the script itself, so read from the terminal.
ask() {
    local prompt="$1" answer
    [[ -r /dev/tty ]] || die "no terminal available for prompts"
    read -r -p "$prompt" answer < /dev/tty
    printf '%s' "$answer"
}

# ── 1. Requirements ───────────────────────────────────────────────────────────

install_git() {
    local sudo=""
    [[ $EUID -ne 0 ]] && command -v sudo >/dev/null 2>&1 && sudo="sudo"
    if   command -v brew    >/dev/null 2>&1; then brew install git
    elif command -v apt-get >/dev/null 2>&1; then $sudo apt-get update && $sudo apt-get install -y git
    elif command -v dnf     >/dev/null 2>&1; then $sudo dnf install -y git
    elif command -v pacman  >/dev/null 2>&1; then $sudo pacman -S --noconfirm git
    elif command -v zypper  >/dev/null 2>&1; then $sudo zypper install -y git
    else die "could not install git automatically — please install it and re-run"
    fi
}

if ! command -v git >/dev/null 2>&1; then
    info "Installing git"
    install_git
fi

if ! command -v uv >/dev/null 2>&1; then
    info "Installing uv"
    curl -LsSf https://astral.sh/uv/install.sh | sh
    # The uv installer puts the binary here; make it visible to this script.
    export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
fi
command -v uv >/dev/null 2>&1 || die "uv was installed but is not on PATH — open a new shell and re-run"

# ── 2. Project name and type ──────────────────────────────────────────────────

NAME="$(ask "Project name: ")"
if [[ ! "$NAME" =~ ^[A-Za-z][A-Za-z0-9_-]*$ ]]; then
    die "name must start with a letter and contain only letters, digits, '-' or '_'"
fi

TYPE="$(ask "Drivetrain type [d]ifferential / [m]ecanum: ")"
TYPE="$(tr '[:upper:]' '[:lower:]' <<< "$TYPE")"
case "$TYPE" in
    d|differential) TYPE="differential" ;;
    m|mecanum)      TYPE="mecanum" ;;
    *) die "type must be 'd' (differential) or 'm' (mecanum)" ;;
esac

# ── 3. Clone ──────────────────────────────────────────────────────────────────

[[ -w . ]] || die "no write permission in $(pwd) — run this from a directory you own"
[[ ! -e "$NAME" ]] || die "'$NAME' already exists in $(pwd)"

info "Cloning the $TYPE example into ./$NAME"
git clone --single-branch -b "$TYPE" "$REPO_URL" "$NAME" || die "clone failed"
cd "$NAME"

# ── 4. Rename the project ─────────────────────────────────────────────────────

# Python package names are conventionally lowercase.
PY_NAME="$(tr '[:upper:]' '[:lower:]' <<< "$NAME")"

info "Renaming project to $NAME"
# `-i.bak` is the one spelling of in-place edit that works on both GNU and BSD sed.
sed -i.bak -E "s/^name = \".*\"/name = \"$PY_NAME\"/" pyproject.toml
sed -i.bak -E "s/^name: .*/name: $NAME/" raccoon.project.yml

# ── 5. Fresh repository and uuid ──────────────────────────────────────────────

info "Creating a fresh git repository"
rm -rf .git
# The example ignores uv.lock; the new project should commit its lockfile.
if [[ -f .gitignore ]]; then
  sed -i.bak -E '/^\/?uv\.lock[[:space:]]*$/d' .gitignore
  rm -f .gitignore.bak
fi
git init -q -b main

if   [[ -r /proc/sys/kernel/random/uuid ]]; then UUID="$(cat /proc/sys/kernel/random/uuid)"
elif command -v uuidgen >/dev/null 2>&1;    then UUID="$(uuidgen | tr '[:upper:]' '[:lower:]')"
else                                             UUID="$(uv run --no-project python -c 'import uuid; print(uuid.uuid4())')"
fi
sed -i.bak -E "s/^uuid: .*/uuid: $UUID/" raccoon.project.yml
rm -f pyproject.toml.bak raccoon.project.yml.bak

# ── 6. Environment and first commit ───────────────────────────────────────────

info "Creating the virtual environment (uv sync)"
uv sync

info "Creating the initial commit"
git add -A
# Always author the first commit as raccoon-setup, whatever the local git identity is.
git -c user.name="raccoon-setup" -c user.email="raccoon-setup@localhost" commit -q -m "Initial Commit"

info "Done! Next steps:"
echo "    cd $NAME"
echo "    raccoon connect <PI_ADDRESS>"
echo "    raccoon run"
