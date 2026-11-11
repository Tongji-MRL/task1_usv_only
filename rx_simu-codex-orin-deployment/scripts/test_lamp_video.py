#!/usr/bin/env python3
"""Run the unchanged lamp colour and lamp state code on frames taken from a video.

Answers one question on real footage: do survey_core.classify_roi (HSV pixel
count) and landmark_map.LampClassifier (2 s / 4 s / 3 s time rules) tell steady
blue, flashing blue, flashing red and flashing green apart?

On the vessel the LiDAR says where each lamp is. A shore video has no LiDAR, so
this script stands in for it, using every native frame:

1. A compact bright, saturated blob of any hue starts a track.
2. While its lamp is dark the track follows the buoy body under it with local
   optical flow (hand-held footage pans and zooms); when the lamp is lit again
   the track snaps back onto it.
   This also finds coloured things that are not lamps; they are reported too.

The box is therefore centred on the lamp, which is the best case for the
LiDAR projection. Then, only on the extracted frames (--fps, which is what
taking frames from a stream does), each box goes through classify_roi and the
label into LampClassifier, both with the production defaults.

Unlit buoys are never found (nothing lights up), so OFF is not tested here.
A track that leaves the picture ends; if the camera returns, a new one starts.

Usage (repository root):
    python3 scripts/test_lamp_video.py VIDEO [--fps 10] [--out DIR]
"""
import argparse
import json
import math
import shutil
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/robotx_task1'))

from robotx_task1 import survey_core as core  # noqa: E402
from robotx_task1.landmark_map import NAMES, UNKNOWN, LampClassifier  # noqa: E402

BGR = dict(red=(0, 0, 255), green=(0, 200, 0), blue=(255, 80, 0), none=(150, 150, 150),
           blocked=(0, 200, 255), out=(40, 40, 40))
FLOW = dict(winSize=(21, 21), maxLevel=3)


def read_frames(path):
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise SystemExit(f'Cannot open {path}')
    rate = capture.get(cv2.CAP_PROP_FPS)
    frames = []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        frames.append(frame)
    if len(frames) < 2 or not rate > 0:
        raise SystemExit('Video has no usable frames or frame rate')
    return frames, float(rate)


def flow(previous, current, corners):
    """Corner displacements that survive a forward-backward check, as (start, end)."""
    if corners is None or len(corners) < 4:
        return None
    moved, forward, _ = cv2.calcOpticalFlowPyrLK(previous, current, corners, None, **FLOW)
    back, backward, _ = cv2.calcOpticalFlowPyrLK(current, previous, moved, None, **FLOW)
    good = ((forward.reshape(-1) == 1) & (backward.reshape(-1) == 1) &
            (np.linalg.norm((back-corners).reshape(-1, 2), axis=1) < 1.0))
    if good.sum() < 4:
        return None
    return corners.reshape(-1, 2)[good], moved.reshape(-1, 2)[good]


def camera_step(previous, current):
    """Similarity previous -> current frame (3x3) from the whole picture, or None."""
    pairs = flow(previous, current, cv2.goodFeaturesToTrack(previous, 300, 0.03, 8))
    if pairs is None or len(pairs[0]) < 12:
        return None
    step, _ = cv2.estimateAffinePartial2D(pairs[0], pairs[1], method=cv2.RANSAC, ransacReprojThreshold=1.5)
    return None if step is None else np.vstack([step, [0., 0., 1.]])


def follow(previous, current, uv, step, scale):
    """Move a lamp position with the buoy body below it; the camera motion is the fallback."""
    reach = 22.0*scale
    height, width = previous.shape
    u0, u1 = int(max(0, uv[0]-reach)), int(min(width, uv[0]+reach))
    v0, v1 = int(max(0, uv[1]-0.6*reach)), int(min(height, uv[1]+2.6*reach))
    if u1-u0 >= 8 and v1-v0 >= 8:
        mask = np.zeros(previous.shape, np.uint8)
        mask[v0:v1, u0:u1] = 255
        pairs = flow(previous, current, cv2.goodFeaturesToTrack(previous, 30, 0.05, 3, mask=mask))
        if pairs is not None:
            return uv+np.median(pairs[1]-pairs[0], axis=0)
    if step is not None:
        return step[:2, :2] @ uv+step[:2, 2]
    return uv


def lit_blobs(rgb, args):
    """Centres and areas of connected bright, saturated pixels of any hue.

    Only the saturation and value thresholds of the colour test are used: a lamp
    whose hue lies outside the production intervals must still be followed, so
    that it shows up in the result as a lamp labelled 'none'.
    """
    _, saturation, value = core.hue_saturation_value(rgb)
    mask = (saturation >= args.s_min) & (value >= args.v_min)
    count, _, stats, centres = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    return [(centres[i], int(stats[i, cv2.CC_STAT_AREA])) for i in range(1, count)]


def hue_spread(hues):
    """Circular 10 %, 50 % and 90 % points of hue angles in degrees, or None."""
    if not len(hues):
        return None
    angles = np.radians(hues)
    centre = math.degrees(math.atan2(np.sin(angles).mean(), np.cos(angles).mean()))
    offset = (np.asarray(hues)-centre+180.) % 360.-180.
    return [int(round((centre+np.percentile(offset, q)) % 360.)) for q in (10, 50, 90)]


def find_tracks(frames, rate, args):
    """Lamp candidates with a pixel position for every frame they are in view.

    Returns (tracks, zoom per frame relative to the first frame, share of failed camera steps).
    """
    live, done, zoom, zooms, failed, previous = [], [], 1.0, [], 0, None
    for index, frame in enumerate(frames):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if previous is not None:
            step = camera_step(previous, gray)
            if step is None:
                failed += 1
            else:
                zoom *= math.sqrt(abs(np.linalg.det(step[:2, :2])))
            for track in live:
                if track['hits'] >= args.min_hits:
                    track['uv'] = follow(previous, gray, track['uv'], step, zoom)
                elif step is not None:  # Unconfirmed glints are many: camera motion only.
                    track['uv'] = step[:2, :2] @ track['uv']+step[:2, 2]
        zooms.append(zoom)
        height, width = gray.shape
        blobs = [centre for centre, area in lit_blobs(frame[:, :, ::-1], args)
                 if area >= args.min_pixels and area/zoom**2 <= args.max_area]
        gate = args.gate*zoom
        pairs = sorted((math.dist(t['uv'], b), i, j) for i, t in enumerate(live)
                       for j, b in enumerate(blobs) if math.dist(t['uv'], b) <= gate)
        used_tracks, used_blobs = set(), set()
        for _, i, j in pairs:
            if i in used_tracks or j in used_blobs:
                continue
            used_tracks.add(i)
            used_blobs.add(j)
            live[i]['uv'] = np.array(blobs[j], float)  # Snap back onto the lit lamp.
            live[i]['hits'] += 1
            live[i]['lit'] = index
        for j, blob in enumerate(blobs):
            if j not in used_blobs and all(math.dist(t['uv'], blob) > gate for t in live):
                live.append(dict(uv=np.array(blob, float), hits=1, lit=index, path={}))
        keep = []
        for track in live:
            u, v = track['uv']
            inside = 0 <= u < width and 0 <= v < height
            # A candidate that never lights up again within 1.5 s was a glint.
            stale = track['hits'] < args.min_hits and index-track['lit'] > 1.5*rate
            if inside and not stale:
                track['path'][index] = track['uv'].copy()
                keep.append(track)
            elif track['hits'] >= args.min_hits:
                done.append(track)
        live = keep
        previous = gray
    tracks = [t for t in done+live if t['hits'] >= args.min_hits]
    tracks.sort(key=lambda t: (min(t['path']), t['path'][min(t['path'])][0]))
    for number, track in enumerate(tracks, 1):
        track['id'] = f'T{number}'
    return tracks, zooms, failed/(len(frames)-1)


def box_at(track, index, zoom, shape, args):
    """Pixel box (u0, v0, u1, v1) around the track in frame `index`, or None if out of view."""
    if index not in track['path']:
        return None
    u, v = track['path'][index]
    half = max(2.0, args.roi/2*zoom)
    height, width = shape[:2]
    if not (half <= u < width-half and half <= v < height-half):
        return None
    return (int(round(u-half)), int(round(v-half)), int(round(u+half)), int(round(v+half)))


def phase_runs(samples):
    """Durations of the complete lit and dark phases in a (time, label) series."""
    lit, dark, start, clean = [], [], 0, False
    for i in range(1, len(samples)+1):
        end = i == len(samples)
        gap = not end and samples[i][0]-samples[i-1][0] > 0.6  # Out of view in between.
        change = not end and not gap and (samples[i][1] == 'none') != (samples[start][1] == 'none')
        if end or gap or change:
            # A phase counts only if it both began and ended with a label change.
            if clean and change:
                (dark if samples[start][1] == 'none' else lit).append(samples[i][0]-samples[start][0])
            start, clean = i, change
    return lit, dark


def classify(frames, picked, rate, zooms, tracks, args):
    """Feed every extracted frame through classify_roi and LampClassifier."""
    for track in tracks:
        track.update(lamp=LampClassifier(), samples=[], decided=None, blocked=0, white=0, history=[],
                     changes=[], hues=[])
    drawn = []
    for index in picked:
        frame, time = frames[index], index/rate
        rgb = frame[:, :, ::-1]
        boxes = {t['id']: box_at(t, index, zooms[index], frame.shape, args) for t in tracks}
        boxes = {name: box for name, box in boxes.items() if box is not None}
        blocked = set() if args.no_overlap_rule else core.overlapping(boxes)
        canvas = frame.copy()
        for track in tracks:
            name = track['id']
            if name not in boxes:
                track['history'].append('out')
                continue
            box = boxes[name]
            if name in blocked:  # Production rule: two lamps in one region, neither is judged.
                track['blocked'] += 1
                label = 'blocked'
            else:
                crop = rgb[box[1]:box[3], box[0]:box[2]]
                result = core.classify_roi(crop, args.s_min, args.v_min, args.min_pixels)
                label = result['label']
                # Diagnostic only: the hue of what classify_roi treats as lit pixels.
                hue, saturation, value = core.hue_saturation_value(crop)
                lit = (saturation >= args.s_min) & (value >= args.v_min)
                if lit.sum() >= args.min_pixels:
                    track['hues'].append(hue[lit])
                track['white'] += result['white']
                track['samples'].append((time, label))
                if track['lamp'].add(time, label):
                    if track['decided'] is None:
                        track['decided'] = time
                    track['changes'].append((round(time, 2), NAMES[track['lamp'].state]))
            track['history'].append(label)
            cv2.rectangle(canvas, box[:2], box[2:], BGR[label], 1)
            text = f"{name} {NAMES[track['lamp'].state]}"
            for colour, width in (((0, 0, 0), 3), ((255, 255, 255), 1)):
                cv2.putText(canvas, text, (box[0], box[1]-4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, width,
                            cv2.LINE_AA)
        cv2.putText(canvas, f't={time:5.2f}s', (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    cv2.LINE_AA)
        drawn.append(canvas)
    return drawn


def summarize(track, rate):
    samples = track['samples']
    counts = {k: sum(1 for _, label in samples if label == k) for k in ('red', 'green', 'blue', 'none')}
    lit, dark = phase_runs(samples)
    first = min(track['path'])
    return dict(
        id=track['id'], first_seen_s=round(first/rate, 2), last_seen_s=round(max(track['path'])/rate, 2),
        first_pixel=[round(float(v), 1) for v in track['path'][first]], state=NAMES[track['lamp'].state],
        decided_after_s=None if track['decided'] is None else round(track['decided']-samples[0][0], 2),
        changes=track['changes'], frames=len(samples), blocked_frames=track['blocked'],
        share={k: round(v/len(samples), 3) if samples else None for k, v in counts.items()},
        lit_phases_s=[round(r, 2) for r in lit], dark_phases_s=[round(r, 2) for r in dark],
        lit_hue_deg_p10_p50_p90=hue_spread(np.concatenate(track['hues']) if track['hues'] else []),
        clipped_white_pixels=track['white'])


def timeline(frames, picked, zooms, tracks, args, cell=36):
    """One row per track: the box content of the extracted frames over the label colour."""
    columns = list(range(0, len(picked), max(1, math.ceil(len(picked)/64))))
    sheet = np.full((len(tracks)*(cell+14)+4, 60+len(columns)*cell, 3), 30, np.uint8)
    for row, track in enumerate(tracks):
        top = row*(cell+14)+2
        cv2.putText(sheet, track['id'], (4, top+cell//2+4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    cv2.LINE_AA)
        for column, k in enumerate(columns):
            index, left = picked[k], 60+column*cell
            box = box_at(track, index, zooms[index], frames[index].shape, args)
            if box is not None:
                crop = frames[index][box[1]:box[3], box[0]:box[2]]
                sheet[top:top+cell, left:left+cell] = cv2.resize(crop, (cell, cell),
                                                                 interpolation=cv2.INTER_NEAREST)
            sheet[top+cell+1:top+cell+10, left:left+cell-1] = BGR[track['history'][k]]
    return sheet


def save_image(path, image):
    """cv2.imwrite cannot write to non-ASCII Windows paths."""
    ok, data = cv2.imencode(Path(path).suffix, image)
    if not ok:
        raise SystemExit(f'Cannot encode {path}')
    data.tofile(str(path))


def save_video(path, images, rate):
    height, width = images[0].shape[:2]
    with tempfile.TemporaryDirectory() as folder:  # Same limitation as imwrite.
        temporary = Path(folder)/'annotated.mp4'
        writer = cv2.VideoWriter(str(temporary), cv2.VideoWriter_fourcc(*'mp4v'), rate, (width, height))
        for image in images:
            writer.write(image)
        writer.release()
        if not temporary.exists() or temporary.stat().st_size == 0:
            return False
        shutil.move(str(temporary), str(path))
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('video')
    parser.add_argument('--fps', type=float, default=10.0, help='frames per second handed to the classifier')
    parser.add_argument('--out', help='output directory; default runs/lamp_video_<name>_<fps>fps')
    parser.add_argument('--roi', type=float, default=24.0, help='box side, pixels at the zoom of the first frame')
    parser.add_argument('--gate', type=float, default=10.0, help='same lamp if closer than this, first-frame px')
    parser.add_argument('--max-area', type=float, default=600.0, help='largest lamp blob, first-frame px^2')
    parser.add_argument('--min-hits', type=int, default=8, help='lit native frames before a place becomes a track')
    parser.add_argument('--no-overlap-rule', action='store_true', help='judge boxes even if they overlap')
    # Production defaults of buoy_mapper; change only to study the thresholds.
    parser.add_argument('--s-min', type=float, default=0.45)
    parser.add_argument('--v-min', type=float, default=0.45)
    parser.add_argument('--min-pixels', type=int, default=4)
    args = parser.parse_args()

    frames, rate = read_frames(args.video)
    used = min(args.fps, rate)
    picked = sorted({min(len(frames)-1, int(round(k*rate/used))) for k in range(int(len(frames)*used/rate)+1)})
    tracks, zooms, failed = find_tracks(frames, rate, args)
    drawn = classify(frames, picked, rate, zooms, tracks, args)

    out = Path(args.out) if args.out else ROOT/'runs'/f'lamp_video_{Path(args.video).stem[:12]}_{used:.3g}fps'
    out.mkdir(parents=True, exist_ok=True)
    rows = [summarize(t, rate) for t in tracks]
    report = dict(video=str(args.video), native_fps=round(rate, 3), classified_fps=round(used, 3),
                  duration_s=round(len(frames)/rate, 2), classified_frames=len(picked),
                  camera_step_failed_share=round(failed, 3), zoom_range=[round(min(zooms), 2), round(max(zooms), 2)],
                  parameters={k: v for k, v in vars(args).items() if k not in ('video', 'out')}, tracks=rows)
    (out/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    if tracks:
        save_image(out/'timeline.png', timeline(frames, picked, zooms, tracks, args))
    for name, k in (('first', 0), ('middle', len(drawn)//2), ('last', len(drawn)-1)):
        save_image(out/f'annotated_{name}.jpg', drawn[k])
    video_ok = save_video(out/'annotated.mp4', drawn, used)

    print(f'{len(frames)} frames at {rate:.2f} fps, {len(picked)} classified at {used:.3g} fps; '
          f'zoom {min(zooms):.2f}-{max(zooms):.2f}; camera step failed on {failed:.1%} of the frames')
    print(f"{'id':4} {'seen s':>11} {'u':>5} {'v':>5} {'frames':>6} {'block':>5} {'red':>5} {'green':>5} "
          f"{'blue':>5} {'none':>5} {'lit s':>6} {'dark s':>6} {'hue 10-50-90%':>13}  {'state':12} decisions")
    for row in rows:
        share = row['share']
        cells = ' '.join(f'{share[k]:5.2f}' if share[k] is not None else '    -'
                         for k in ('red', 'green', 'blue', 'none'))
        lit = f"{np.median(row['lit_phases_s']):6.2f}" if row['lit_phases_s'] else '     -'
        dark = f"{np.median(row['dark_phases_s']):6.2f}" if row['dark_phases_s'] else '     -'
        # Every latched decision with its video time: a lamp first taken for OFF must be visible.
        after = ' -> '.join(f'{name}@{time:.1f}s' for time, name in row['changes']) or '-'
        hues = row['lit_hue_deg_p10_p50_p90']
        hues = '-' if hues is None else '{}-{}-{}'.format(*hues)
        print(f"{row['id']:4} {row['first_seen_s']:5.1f}-{row['last_seen_s']:<5.1f} {row['first_pixel'][0]:5.0f} "
              f"{row['first_pixel'][1]:5.0f} {row['frames']:6d} {row['blocked_frames']:5d} {cells} {lit} {dark} "
              f"{hues:>13}  {row['state']:12} {after}")
    undecided = sum(row['state'] == NAMES[UNKNOWN] for row in rows)
    print(f'{len(rows)} tracks, {undecided} still unknown. Output: {out}'
          + ('' if video_ok else ' (annotated.mp4 could not be written)'))


if __name__ == '__main__':
    main()
