# TOOLS

The RPOD-Software repository contains the following tools to support development and testing.
- Image Simulator (```sim/vbn_simulator.py```)
- VBN Evaluator (```eval/vbn_evaluator.py```)
- V4L2 Media Graph Setup (```v4l2_media_setup.sh```)
- Camera Calibration (TO BE ADDED)

---
## V4L2 MEDIA GRAPH SETUP

It is important the configure the media graph correctly before we can take images from our camera using V4L2 API calls directly from rp1-cfe-ch0.
Snippet from ```v4l2_media_setup.sh``` is shown below.
Modify the configurations and run the script
Verify the correct settings and then proceed with VBN pipeline testing.

```bash
# =========================
# USER CONFIGURATION
# =========================
WIDTH=1280
HEIGHT=800

EXPOSURE=100
GAIN=16

# Optional: set vertical blanking. Leave empty to not touch it.
VBLANK=""   # e.g. "20"

# FOURCC has a trailing space
PIXFMT="Y16 "

# Optional FPS control via vertical blanking (recommended)
TARGET_FPS="80"   # e.g. "60" ; leave empty to skip

```