<div align="center">

<img src="https://raw.githubusercontent.com/htl-stp-ecer/.github/main/profile/raccoon-logo.svg" alt="raccoon-example" width="100"/>

# raccoon-example

**A clean, fully-commented reference robot built with RaccoonOS.**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
![Python](https://img.shields.io/badge/Python-3.13-3776AB?logo=python&logoColor=ffdd54)
![Platform](https://img.shields.io/badge/Platform-KIPR%20Wombat-orange)

> 📖 **Full documentation at [raccoon-docs.pages.dev](https://raccoon-docs.pages.dev/)**

</div>

---

## About this repository

This repository is the starting point for new RaccoonOS robot projects. The example
robot itself comes in two variants, and each one lives on **its own branch**:

| Branch | Drivetrain | Contents |
|:-------|:-----------|:---------|
| [`differential`](https://github.com/htl-stp-ecer/raccoon-example/tree/differential) | Two-wheeled (differential drive) | Complete example project |
| [`mecanum`](https://github.com/htl-stp-ecer/raccoon-example/tree/mecanum) | Four-wheeled (mecanum, "omniwheeled") | Complete example project |

**The `main` branch contains no robot code.** It only holds the setup scripts below
(plus this README and the license). To see how a robot project is structured, browse
one of the branches above — the README there explains the layout, the concepts it
demonstrates, and how to adapt it to your robot.

Other drivetrain types are currently not supported by the template.

---

## Create a new project

The setup script does everything for you. Open a terminal in the folder you'd like to
create the project in and run:

**Linux / macOS**

```bash
curl -LsSf https://raw.githubusercontent.com/htl-stp-ecer/raccoon-example/main/raccoon-setup.sh | bash
```

**Windows (PowerShell)**

```powershell
irm https://raw.githubusercontent.com/htl-stp-ecer/raccoon-example/main/raccoon-setup.ps1 | iex
```

It will:

1. Install **git** and **uv** if they are missing
2. Ask for a **project name** and a **drivetrain type** (`d` for differential, `m` for mecanum; the full words work too)
3. Clone the matching branch of this repository into a new folder named after your project
   (it aborts if the folder is not writable or the name is already taken)
4. Rename the project in `pyproject.toml` and `raccoon.project.yml`
5. Replace the example's git history with a fresh repository and give the project its own
   unique `uuid`
6. Run `uv sync` to create the virtual environment
7. Create the first commit, titled *Initial Commit*

Afterwards, activate the virtual environment, connect to your Wombat and run:

```bash
cd <your-project>
source .venv/bin/activate        # Windows PowerShell: .venv\Scripts\Activate.ps1
raccoon connect <PI_ADDRESS>
raccoon run
```

New to RaccoonOS? Follow the
[Quick Start](https://raccoon-docs.pages.dev/00-quick-start) from flashing the SD card
to a driving robot.

---

## Related Repositories

| Repository | What it is |
|:-----------|:-----------|
| [raccoon-lib](https://github.com/htl-stp-ecer/raccoon-lib) | The core robotics library |
| [raccoon-cli](https://github.com/htl-stp-ecer/raccoon-cli) | `raccoon run`, `raccoon calibrate`, … |
| [documentation](https://github.com/htl-stp-ecer/documentation) | Full platform docs |

---

## License

The example code in this repository is MIT licensed — see [LICENSE](LICENSE).

You can freely use, modify, and build on it for your competition robot
**without having to publish your own robot code based on this repo**.
