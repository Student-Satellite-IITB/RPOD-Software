#!/usr/bin/env bash
set -euo pipefail

# =========================
# USER CONFIGURATION
# =========================
WIDTH=1280
HEIGHT=800

EXPOSURE=$1
GAIN=16

# Optional: set vertical blanking. Leave empty to not touch it.
VBLANK=""   # e.g. "20"

# FOURCC has a trailing space
PIXFMT="Y16 "

# Optional FPS control via vertical blanking (recommended)
TARGET_FPS="80"   # e.g. "60" ; leave empty to skip

# =========================
die() { echo "ERROR: $*" >&2; exit 1; }

need_root() {
  if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
    die "Run as root: sudo $0"
  fi
}

find_rp1_media() {
  for dev in /dev/media*; do
    [[ -e "$dev" ]] || continue
    if media-ctl -d "$dev" -p 2>/dev/null | grep -qE '^driver[[:space:]]+rp1-cfe|^model[[:space:]]+rp1-cfe'; then
      echo "$dev"
      return 0
    fi
  done
  return 1
}

# Extract first sensor entity name like: "ov9281 11-0060"
find_sensor_entity_name() {
  local media_dev="$1"
  media-ctl -d "$media_dev" -p \
    | sed -n 's/^- entity [0-9]\+: \(ov9281[^()]*\) (.*/\1/p' \
    | head -n1
}

# Given exact entity name, return its device node name (/dev/videoX or /dev/v4l-subdevY)
get_devnode_for_entity() {
  local media_dev="$1"
  local entity_name="$2"

  media-ctl -d "$media_dev" -p | awk -v ent="$entity_name" '
    /^- entity / {
      inside = (index($0, ": " ent " (") > 0)
    }
    inside && /device node name/ { print $NF; exit }
  '
}

set_fps_via_vblank() {
  local subdev="$1"
  local width="$2"
  local height="$3"
  local target_fps="$4"

  # Read pixel_rate (int64) and hblank limits, vblank limits
  local pixel_rate hblank vmin vmax
  pixel_rate="$(v4l2-ctl -d "$subdev" --get-ctrl=pixel_rate 2>/dev/null | awk -F': ' '{print $2}')"
  hblank="$(v4l2-ctl -d "$subdev" --get-ctrl=horizontal_blanking 2>/dev/null | awk -F': ' '{print $2}')"

  # Parse vblank min/max from --all (easy + robust)
  vmin="$(v4l2-ctl -d "$subdev" --all 2>/dev/null | awk '/vertical_blanking/{for(i=1;i<=NF;i++){if($i~"min="){sub("min=","",$i);print $i;exit}}}')"
  vmax="$(v4l2-ctl -d "$subdev" --all 2>/dev/null | awk '/vertical_blanking/{for(i=1;i<=NF;i++){if($i~"max="){sub("max=","",$i);print $i;exit}}}')"

  [[ -n "$pixel_rate" && -n "$hblank" && -n "$vmin" && -n "$vmax" ]] || {
    echo "  WARN: Could not read pixel_rate/hblank/vblank limits; skipping FPS set."
    return 0
  }

  # Compute required total lines:
  # total_lines = pixel_rate / (target_fps * total_pixels_per_line)
  # where total_pixels_per_line = (width + hblank)
  # vblank = total_lines - height
  local total_pixels_per_line total_lines vblank
  total_pixels_per_line=$(( width + hblank ))

  # integer math with rounding:
  total_lines="$(awk -v pr="$pixel_rate" -v fps="$target_fps" -v ppl="$total_pixels_per_line" \
    'BEGIN{print int((pr/(fps*ppl))+0.5)}')"

  vblank=$(( total_lines - height ))

  # clamp to driver limits
  if (( vblank < vmin )); then vblank=$vmin; fi
  if (( vblank > vmax )); then vblank=$vmax; fi

  echo "  pixel_rate=$pixel_rate, hblank=$hblank -> setting vertical_blanking=$vblank for target_fps=$target_fps"
  v4l2-ctl -d "$subdev" --set-ctrl="vertical_blanking=${vblank}" >/dev/null
}

compute_expected_fps() {
  local subdev="$1"
  local width="$2"
  local height="$3"

  local pixel_rate hblank vblank
  pixel_rate="$(v4l2-ctl -d "$subdev" --get-ctrl=pixel_rate 2>/dev/null | awk -F': ' '{print $2}')"
  hblank="$(v4l2-ctl -d "$subdev" --get-ctrl=horizontal_blanking 2>/dev/null | awk -F': ' '{print $2}')"
  vblank="$(v4l2-ctl -d "$subdev" --get-ctrl=vertical_blanking 2>/dev/null | awk -F': ' '{print $2}')"

  if [[ -z "$pixel_rate" || -z "$hblank" || -z "$vblank" ]]; then
    echo ""
    return 0
  fi

  # fps = pixel_rate / ((W+hblank)*(H+vblank))
  awk -v pr="$pixel_rate" -v w="$width" -v hb="$hblank" -v h="$height" -v vb="$vblank" \
    'BEGIN{fps = pr / ((w+hb)*(h+vb)); printf "%.3f", fps}'
}

measure_fps() {
  local video_node="$1"
  local count="$2"
  local out fps
  out="$(v4l2-ctl -d "$video_node" --stream-mmap --stream-count="$count" --stream-to=/dev/null 2>&1 || true)"
  fps="$(echo "$out" | grep -oE 'fps:[[:space:]]*[0-9]+(\.[0-9]+)?' | head -n1 | grep -oE '[0-9]+(\.[0-9]+)?' || true)"
  [[ -n "$fps" ]] || fps="$(echo "$out" | grep -oE '[0-9]+(\.[0-9]+)?[[:space:]]*fps' | head -n1 | grep -oE '[0-9]+(\.[0-9]+)?' || true)"
  echo "$fps"
}

# =========================
# Main
# =========================
need_root

echo "v4l2 devices (sanity):"
v4l2-ctl --list-devices || true
echo

MEDIA_DEV="$(find_rp1_media || true)"
[[ -n "${MEDIA_DEV:-}" ]] || die "Could not find rp1-cfe media device under /dev/media*"
echo "Using media device: $MEDIA_DEV"

SENSOR_ENTITY="$(find_sensor_entity_name "$MEDIA_DEV" || true)"
[[ -n "${SENSOR_ENTITY:-}" ]] || die "Could not find ov9281 sensor entity on $MEDIA_DEV"
echo "Sensor entity: \"$SENSOR_ENTITY\""

CSI_ENTITY="csi2"
CH0_ENTITY="rp1-cfe-csi2_ch0"

VIDEO_NODE="$(get_devnode_for_entity "$MEDIA_DEV" "$CH0_ENTITY" || true)"
[[ -n "${VIDEO_NODE:-}" ]] || die "Could not find video node for entity '$CH0_ENTITY' on $MEDIA_DEV"

SENSOR_SUBDEV="$(get_devnode_for_entity "$MEDIA_DEV" "$SENSOR_ENTITY" || true)"
[[ -n "${SENSOR_SUBDEV:-}" ]] || die "Could not find subdev node for sensor '$SENSOR_ENTITY' on $MEDIA_DEV"

echo "CH0 video node: $VIDEO_NODE"
echo "Sensor subdev : $SENSOR_SUBDEV"
echo

echo "Resetting media links/routes..."
media-ctl -d "$MEDIA_DEV" -r

echo "Enabling link: csi2:4 -> rp1-cfe-csi2_ch0:0"
media-ctl -d "$MEDIA_DEV" -l "'${CSI_ENTITY}':4->'${CH0_ENTITY}':0[1]"

echo "Setting formats:"
media-ctl -d "$MEDIA_DEV" -V "\"${SENSOR_ENTITY}\":0 [fmt:Y10_1X10/${WIDTH}x${HEIGHT} field:none]"
media-ctl -d "$MEDIA_DEV" -V "\"${CSI_ENTITY}\":0 [fmt:Y10_1X10/${WIDTH}x${HEIGHT} field:none colorspace:raw]"
media-ctl -d "$MEDIA_DEV" -V "\"${CSI_ENTITY}\":4 [fmt:Y16_1X16/${WIDTH}x${HEIGHT} field:none colorspace:raw]"

echo "Setting video node format on $VIDEO_NODE: ${WIDTH}x${HEIGHT} pixelformat=${PIXFMT}"
v4l2-ctl -d "$VIDEO_NODE" --set-fmt-video="width=${WIDTH},height=${HEIGHT},pixelformat=${PIXFMT}"

echo "Setting sensor controls on $SENSOR_SUBDEV: exposure=$EXPOSURE, analogue_gain=$GAIN"
if [[ -n "${VBLANK}" ]]; then
  v4l2-ctl -d "$SENSOR_SUBDEV" --set-ctrl="exposure=${EXPOSURE},analogue_gain=${GAIN},vertical_blanking=${VBLANK}"
else
  v4l2-ctl -d "$SENSOR_SUBDEV" --set-ctrl="exposure=${EXPOSURE},analogue_gain=${GAIN}"
fi
# FPS setting
if [[ -n "${TARGET_FPS}" ]]; then
  echo "Setting FPS via vertical blanking..."
  set_fps_via_vblank "$SENSOR_SUBDEV" "$WIDTH" "$HEIGHT" "$TARGET_FPS"
fi

echo
echo "==== Verification ===="
echo "[video fmt]"
v4l2-ctl -d "$VIDEO_NODE" --get-fmt-video || true
echo
echo "[sensor ctrls]"
v4l2-ctl -d "$SENSOR_SUBDEV" --get-ctrl=exposure,analogue_gain,horizontal_blanking,vertical_blanking || true
echo 
echo "[fps verification]"
exp_fps="$(compute_expected_fps "$SENSOR_SUBDEV" "$WIDTH" "$HEIGHT")"
if [[ -n "$exp_fps" ]]; then
  echo "  expected_fps (from pixel_rate/hblank/vblank, using ${WIDTH}x${HEIGHT}) = ${exp_fps}"
else
  echo "  expected_fps = (could not compute)"
fi

# optional actual measurement (short)
meas_fps="$(measure_fps "$VIDEO_NODE" 180)"
if [[ -n "$meas_fps" ]]; then
  echo "  measured_fps (v4l2-ctl stream) = ${meas_fps}"
else
  echo "  measured_fps = (could not parse)"
fi
echo
echo "Done."
