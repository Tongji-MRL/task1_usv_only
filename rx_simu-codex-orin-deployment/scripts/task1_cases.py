"""User-confirmed single-change competition cases and separate stress cases."""


def case_key(case):
    key=f"s{case['seed']}_c{case['changes']}_{case['dynamic']}"
    if case.get('course','baseline')!='baseline':key=case['course']+'_'+key
    if case.get('wind',0) or case.get('wave_gain',0):
        key+=f"_wind{case.get('wind',0):g}_wave{case.get('wave_gain',0):g}"
    return key


def cases(group='all', stress=False):
    dynamics={'base':('none',),'dynamic':('cross','block'),'all':('none','cross','block')}[group]
    # Prioritize the confirmed match setting, then zero-change controls.
    changes=(2,) if stress else (1,0)
    return [dict(seed=seed,changes=count,dynamic=dynamic)
            for count in changes for seed in (1,2,3) for dynamic in dynamics]


FAULTS=('observation_gap','reordered_observation','mission_exit','cancel','role_change','no_path','late_change',
        'traffic_gap','traffic_exit')
