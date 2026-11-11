#!/usr/bin/env python3
"""Reproduce pinned user-local dependencies and builds without sudo."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
LOCK = json.loads((ROOT/'config/dependencies.lock.json').read_text())
EXTERNAL = ROOT/'external'


def run(command, cwd=ROOT):
    print('+', ' '.join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), cwd=cwd, check=True)


def verified(path, digest):
    return path.exists() and hashlib.sha256(path.read_bytes()).hexdigest() == digest


def dependencies():
    debs=EXTERNAL/'debs';prefix=EXTERNAL/'deps'
    debs.mkdir(parents=True,exist_ok=True);prefix.mkdir(exist_ok=True)
    for item in LOCK['debs']:
        path=debs/item['file']
        if not path.exists():run(['apt-get','download',item['package']+'='+item['version']],debs)
        if not verified(path,item['sha256']):raise RuntimeError('Package checksum mismatch: '+str(path))
        run(['dpkg-deb','-x',path,prefix])
    archive=debs/'egm96-5.tar.bz2'
    if not archive.exists():
        run(['curl','-fL','--retry','3','--connect-timeout','15','--max-time','180',
             'https://downloads.sourceforge.net/project/geographiclib/geoids-distrib/egm96-5.tar.bz2','-o',archive])
    if not verified(archive,LOCK['geoid_sha256']):raise RuntimeError('Geoid checksum mismatch')
    target=prefix/'usr/share/GeographicLib';target.mkdir(parents=True,exist_ok=True)
    with tarfile.open(archive) as package:
        for member in package.getmembers():
            if not (target/member.name).resolve().is_relative_to(target.resolve()) or member.issym() or member.islnk():
                raise RuntimeError('Unsafe archive member')
        package.extractall(target)


def sources():
    firmware=EXTERNAL/'ardupilot'
    local=Path(os.environ.get('ARDUPILOT_SOURCE','/home/zjy/ardurover-usv'))
    if not (firmware/'.git').exists():
        run(['git','clone','--no-hardlinks','--no-checkout',local,firmware])
        run(['git','checkout','--detach',LOCK['ardupilot']],firmware)
        for line in subprocess.check_output(['git','config','-f','.gitmodules','--get-regexp','path'],cwd=firmware,text=True).splitlines():
            key,path=line.split(maxsplit=1)
            run(['git','config',key.removesuffix('.path')+'.url',local/path],firmware)
        run(['git','-c','protocol.file.allow=always','submodule','update','--init','--jobs','4'],firmware)
    revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=firmware,text=True).strip()
    if revision!=LOCK['ardupilot']:raise RuntimeError('Refusing to change an existing firmware checkout')
    # Relative nested URLs must resolve against upstream, not a local mirror path.
    for gm in firmware.glob('modules/**/.gitmodules'):
        result=subprocess.run(['git','config','-f',str(gm),'--get-regexp','path'],capture_output=True,text=True)
        for line in result.stdout.splitlines():
            key,path=line.split(maxsplit=1);candidate=local/gm.parent.relative_to(firmware)/path
            if (candidate/'.git').exists():url=str(candidate)
            else:
                url=subprocess.check_output(['git','config','-f',str(gm),key.removesuffix('.path')+'.url'],text=True).strip()
                if url.startswith('../') and 'CrashDebug' in str(gm):url='https://github.com/adamgreen/'+url[3:]
            run(['git','config',key.removesuffix('.path')+'.url',url],gm.parent)
    run(['git','-c','protocol.file.allow=always','submodule','update','--init','--recursive','--jobs','4'],firmware)
    if subprocess.check_output(['git','status','--porcelain','--untracked-files=no'],cwd=firmware,text=True):
        raise RuntimeError('Refusing to build modified firmware or submodules')
    plugin=EXTERNAL/'ardupilot_gazebo'
    if not (plugin/'.git').exists():
        run(['git','init',plugin]);run(['git','remote','add','origin','https://github.com/ArduPilot/ardupilot_gazebo.git'],plugin)
        run(['git','config','core.sparseCheckout','true'],plugin)
        (plugin/'.git/info/sparse-checkout').write_text('/CMakeLists.txt\n/cmake/\n/include/\n/src/\n/LICENSE\n/README.md\n')
        run(['git','fetch','--depth','1','--filter=blob:none','origin',LOCK['ardupilot_gazebo']],plugin)
        run(['git','checkout','--detach','FETCH_HEAD'],plugin)
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=plugin,text=True).strip()!=LOCK['ardupilot_gazebo']:
        raise RuntimeError('Plugin revision mismatch')
    patch=ROOT/'patches/ardupilot_gazebo_rover44.patch'
    applied=subprocess.run(['git','apply','--reverse','--check',str(patch)],cwd=plugin,capture_output=True).returncode==0
    if not applied:run(['git','apply',patch],plugin)
    if subprocess.check_output(['git','diff','HEAD','--binary'],cwd=plugin)!=patch.read_bytes():
        raise RuntimeError('Unexpected plugin modifications beyond the compatibility patch')


def main():
    EXTERNAL.mkdir(exist_ok=True)
    dependencies();sources()
    run(['./waf','configure','--board','sitl'],EXTERNAL/'ardupilot')
    run(['./waf','rover','-j8'],EXTERNAL/'ardupilot')
    # Run builds inside the same explicit ROS/Garden environment used at runtime.
    run(['bash',str(ROOT/'scripts/build.sh')])


if __name__=='__main__':main()
