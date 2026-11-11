#!/usr/bin/env python3
"""Bundle a finished experiment; retain large rosbag files separately with hashes."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(8*1024*1024),b''):value.update(block)
    return value.hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args()
    run=args.run.resolve();result=json.loads((run/'task1_result.json').read_text())
    session=ROOT/'runs/session.json'
    if session.exists() and Path(json.loads(session.read_text())['run_dir']).resolve()==run:
        raise RuntimeError('Stop recording before exporting an experiment')
    destination=ROOT/'runs/exports'/(run.name+'.zip');destination.parent.mkdir(exist_ok=True)
    if destination.exists():raise FileExistsError('Existing immutable bundle: '+str(destination))
    files=[];external=[]
    for path in sorted(run.rglob('*')):
        if not path.is_file() or path.name=='export_manifest.json' or '.partial.' in path.name:continue
        relative=path.relative_to(run)
        item=dict(path=str(relative),bytes=path.stat().st_size,sha256=digest(path))
        if relative.parts[0]=='sensors' and path.suffix in ('.db3','.mcap'):
            item['absolute_path']=str(path);external.append(item)
        else:files.append(item)
    manifest=dict(run=str(run),task_passed=result['passed'],files=files,external_rosbags=external,
                  note='Bundle includes recorded logs, parameters, source snapshot and generated media. Large raw sensor bags remain at the hashed paths listed separately.')
    (run/'export_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    partial=destination.with_suffix('.partial.zip')
    with zipfile.ZipFile(partial,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
        for item in files+[dict(path='export_manifest.json')]:
            source=run/item['path']
            method=zipfile.ZIP_STORED if source.suffix in ('.mp4','.png','.npz','.gz') else zipfile.ZIP_DEFLATED
            archive.write(source,arcname=run.name+'/'+item['path'],compress_type=method)
    with zipfile.ZipFile(partial) as archive:
        bad=archive.testzip()
        if bad:raise RuntimeError('ZIP integrity failed: '+bad)
    partial.rename(destination)
    output=dict(bundle=str(destination),bytes=destination.stat().st_size,sha256=digest(destination),
                files=len(files)+1,external_rosbags=external)
    destination.with_suffix('.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))


if __name__=='__main__':main()
