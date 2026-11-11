#!/usr/bin/env python3
"""Compact immutable run evidence for review without checking bags into Git."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics

ROOT=Path(__file__).resolve().parents[1]
FILES={'geometry':'lidar_geometry_result.json','station':'stationkeeping_result.json',
       'loiter':'loiter_result.json','gnss':'loiter_gnss_result.json','task1':'task1_result.json',
       'inputs':'fault_inputs_result.json','pause':'pause_result.json','reset':'reset_result.json'}


def main():
    p=argparse.ArgumentParser();p.add_argument('manifest',type=Path);a=p.parse_args()
    manifest=json.loads(a.manifest.read_text());result=dict(status=manifest['status'],runs=[])
    for spec in manifest['runs']:
        run=ROOT/'runs'/spec['run'];path=run/FILES[spec['kind']]
        data=json.loads(path.read_text());item=dict(spec,absolute_run=str(run),result=data,
            result_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        provenance=json.loads((run/'provenance.json').read_text())
        item['provenance']=provenance
        if spec['kind']=='geometry':
            clearing=data['clearing'] if isinstance(data['clearing'],list) else [data['clearing']]
            for c in clearing:
                c['first_zero_occupied_cell_wall_s']=next((t for t,n in c.pop('series') if n==0),None)
            data['clearing']=clearing
            data['cost_definition']='OccupancyGrid >=99, including lethal and inscribed inflation cells'
        if spec['kind']=='loiter' and data.get('measurement_completed'):
            previous=-1e9;curve=[];roll=[];pitch=[];heave=[]
            for line in (run/'loiter.jsonl').read_text().splitlines():
                row=json.loads(line)
                if row.get('phase')=='hold_measurement' and 'truth' in row:
                    if row['sim']-previous>=.49:
                        curve.append([row['sim'],*row['truth'][:2]]);previous=row['sim']
                    if 'truth_rpy' in row:
                        roll.append(row['truth_rpy'][0]);pitch.append(row['truth_rpy'][1]);heave.append(row['truth'][2])
            data['truth_curve_sim_xy']=curve
            if heave:data['measured_body_motion']=dict(
                roll_peak_to_peak_deg=math.degrees(max(roll)-min(roll)),
                pitch_peak_to_peak_deg=math.degrees(max(pitch)-min(pitch)),
                heave_peak_to_peak_m=max(heave)-min(heave),heave_std_m=statistics.pstdev(heave),
                note='Vessel response, not measured wave height or optical calibration.')
        for source,key in [('shutdown_audit.json','shutdown'),('mid360_result.json','sensor_quality'),
                           ('protocol_continuity.json','protocol'),('visualization/task1_annotated.json','video'),
                           ('task1_stationkeeping_result.json','automatic_stationkeeping'),
                           ('dynamic_map_result.json','dynamic_map'),('buoy_map_result.json','buoy_map'),
                           ('graph_audit.json','graph'),('tf_authority.json','tf_authority'),
                           ('diagnostic_termination.json','diagnostic_termination')]:
            if (run/source).exists():
                item[key]=json.loads((run/source).read_text())
                if key=='video':
                    item[key].pop('camera_source_stamps',None)
                    item[key]['metadata_sha256']=hashlib.sha256((run/source).read_bytes()).hexdigest()
        if 'dynamic_map' in item:item['dynamic_map'].pop('samples',None)
        export=ROOT/'runs/exports'/(run.name+'.json')
        if export.exists():item['export']=json.loads(export.read_text())
        result['runs'].append(item)
    output=a.manifest.with_name('evidence.json');output.write_text(json.dumps(result,indent=2)+'\n')
    print(output)


if __name__=='__main__':main()
