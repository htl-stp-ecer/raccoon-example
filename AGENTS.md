# AGENTS.md

This project's agent instructions live in **[`CLAUDE.md`](CLAUDE.md)** — the same
guidance applies to every coding agent (Claude Code, Codex, Cursor, …).

Quick summary (read `CLAUDE.md` for the full contract):

- The library is imported as `import raccoon` (never `libstp`).
- `config/*.yml` is the source of truth; `src/hardware/defs.py` and `robot.py` are
  **generated** by `raccoon run` — edit the YAML, not those files.
- Architecture is layered: **missions** (sequencing) → **steps** (actions) →
  **services** (shared state). Keep each in its lane.
- Prefer built-in `raccoon` primitives; introspect the library before inventing
  calls (`RACCOON_PLATFORM=mock python -c "import raccoon; ..."`).
- Logging and run artifacts: see [`docs/LOGGING.md`](docs/LOGGING.md).
