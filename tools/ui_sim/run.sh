#!/bin/sh
# Builds the touchscreen UI simulator, taps through a whole job with the demo wheel and writes a
# PNG of each screen (run in WSL or Linux):
#   sh tools/ui_sim/run.sh [outdir]          default outdir: docs/ui
# Needs g++ and a Python with Pillow (ESPHome's has it; PYTHON=... to pick another).
set -e
cd "$(dirname "$0")/../.."
OUT="${1:-docs/ui}"
if [ -z "$PYTHON" ]; then
  if [ -x "$HOME/esphome/venv/bin/python" ]; then PYTHON="$HOME/esphome/venv/bin/python"; else PYTHON=python3; fi
fi
C=components/balancer
bin="${TMPDIR:-/tmp}/balancer_ui_sim"
mkdir -p "$OUT"
rm -f "$OUT"/*.ppm "$OUT"/*.png
g++ -std=c++17 -O2 -g -Wall -Wextra -Werror -I"$C" -Itools/ui_sim -o "$bin" tools/ui_sim/sim.cpp \
  $C/ui.cpp $C/ui_screens.cpp $C/ui_kit.cpp $C/ui_canvas.cpp $C/ui_fonts_data.cpp $C/ui_core.cpp \
  $C/dsp_revolution.cpp $C/dsp_measurement.cpp $C/dsp_balance.cpp $C/dsp_job.cpp $C/dsp_sim.cpp
"$bin" "$OUT"
OUT="$OUT" "$PYTHON" - <<'EOF'
import glob, os
from PIL import Image
for path in sorted(glob.glob(os.environ["OUT"] + "/*.ppm")):
    Image.open(path).save(path[:-4] + ".png")
    os.remove(path)
EOF
