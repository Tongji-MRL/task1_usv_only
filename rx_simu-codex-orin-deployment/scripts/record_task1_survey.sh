#!/usr/bin/env bash
# Record the Task 1 sensor survey bag (docs/task1/usv_only_plan.md, step 1).
# Subscribes only: never arms, changes mode or publishes commands. The vessel
# is driven by RC. Usage, after `source scripts/hardware_env.sh`:
#   bash scripts/record_task1_survey.sh [name]
# USV_SURVEY_FORCE=1 records even if the check fails (for diagnosis only).
set -eo pipefail
if [[ -z "${USV_WORKSPACE:-}" ]] || ! command -v ros2 >/dev/null; then
    echo 'First run: source scripts/hardware_env.sh' >&2
    exit 2
fi
USV_SURVEY_NAME="${1:-survey}"
if [[ ! "$USV_SURVEY_NAME" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo 'Name may contain only letters, digits, _ and -.' >&2
    exit 2
fi
USV_RUN="$USV_WORKSPACE/runs/task1_${USV_SURVEY_NAME}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$USV_RUN"
git -C "$USV_WORKSPACE" rev-parse HEAD > "$USV_RUN/revision.txt" 2>/dev/null \
    || echo 'not a git checkout' > "$USV_RUN/revision.txt"
git -C "$USV_WORKSPACE" status --short > "$USV_RUN/git_status.txt" 2>/dev/null || true
if [[ -d "$USV_WORKSPACE/runs/hardware_config" ]]; then
    cp -r "$USV_WORKSPACE/runs/hardware_config" "$USV_RUN/config"
fi

if ! ros2 run robotx_task1 survey_check --json "$USV_RUN/check.json"; then
    if [[ "${USV_SURVEY_FORCE:-0}" != 1 ]]; then
        echo "Check failed; nothing recorded. Result: $USV_RUN/check.json" >&2
        echo 'Fix the FAIL lines, or set USV_SURVEY_FORCE=1 to record for diagnosis.' >&2
        exit 1
    fi
    echo 'USV_SURVEY_FORCE=1: recording despite the failed check.' >&2
fi
mapfile -t USV_SURVEY_TOPICS < <(python3 -c \
    'import json, sys; print("\n".join(json.load(open(sys.argv[1]))["record_topics"]))' \
    "$USV_RUN/check.json")
if [[ "${#USV_SURVEY_TOPICS[@]}" -eq 0 ]]; then
    echo 'No topics to record.' >&2
    exit 1
fi

cat > "$USV_RUN/notes.md" <<'NOTES'
# Task 1 survey notes (fill in on site)

- Date / time / weather / sun direction relative to the camera:
- Camera exposure, gain, white balance (auto or fixed values):
- RTK fix type shown on the ground station (fixed / float / none):
- Buoy height and diameter (measured or estimated):

| Buoy (describe position) | Lamp state (red / green / blue_flash / blue_steady / off) | Notes |
|---|---|---|
|  |  |  |

- Tape or eye estimate of the distance at which the color became clear:
- Anything unusual (wake, other boats, glare, dropouts):
NOTES

df -h "$USV_RUN" | tail -1
echo "Recording to $USV_RUN/bag"
echo "Topics: ${USV_SURVEY_TOPICS[*]}"
echo 'Stop with Ctrl-C, then check: ros2 bag info <bag directory>'
exec ros2 bag record -o "$USV_RUN/bag" "${USV_SURVEY_TOPICS[@]}"
