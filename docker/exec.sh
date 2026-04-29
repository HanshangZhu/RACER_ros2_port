#!/usr/bin/env bash
# Open another bash shell inside the already-running racer container.
set -euo pipefail
exec docker exec -it racer bash
