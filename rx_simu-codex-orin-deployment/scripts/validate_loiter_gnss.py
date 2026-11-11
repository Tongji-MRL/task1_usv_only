#!/usr/bin/env python3
"""Observe native LOITER under GPS_INPUT changes; report gaps without new gates."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from pymavlink import mavutil
from validate_loiter import LoiterRunner


def main():
    rclpy.init();r=LoiterRunner();result={'measurement_completed':False,'profiles':[]}
    try:
        r.ready();client=r.node.create_client(SetParameters,'/gnss_injector/set_parameters')
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Start with --gps-input')
        for name,value in [('LOIT_RADIUS',.3),('WP_SPEED',.3),('ATC_STR_RAT_MAX',23.)]:
            r.set_parameter(name,value)
        r.mav.mav.command_long_send(1,1,511,0,87,500000,0,0,0,0,0)
        r.arm();r.mode('LOITER');r.spin(3.)
        r.wait(lambda:bool(r.targets),5,'Native hold target')
        anchor=r.targets[-1]['xy'];result['original_anchor_xy_m']=anchor
        profiles=[('fixed',6,.02,.02,.1,False,25,0.),('biased_fixed',6,.02,.02,.1,False,30,1.5),
                  ('bias_recover',6,.02,.02,.1,False,25,0.),('float',5,.3,.3,.1,False,25,0.),
                  ('ordinary',3,1.5,1.5,.3,False,30,0.),('no_fix',1,1.5,1.5,.1,False,25,0.),
                  ('lost',1,1.5,1.5,.3,True,35,0.),('recover',6,.02,.02,.1,False,30,0.)]
        for name,fix,sigma,accuracy,delay,drop,duration,bias in profiles:
            values=dict(fix_type=fix,horizontal_sigma=sigma,reported_accuracy=accuracy,delay_s=delay,drop=drop,
                        east_bias=0.,north_bias=0.)
            f=client.call_async(SetParameters.Request(parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in values.items()]))
            r.wait(f.done,5,'GNSS injection profile')
            if not all(v.successful for v in f.result().results):raise RuntimeError(str(f.result()))
            r.phase='gnss_'+name;begin=len(r.rows);start=r.node.get_clock().now().nanoseconds/1e9
            deadline=time.monotonic()+duration*3;previous_bias=-1.;bias_updates=[]
            while r.node.get_clock().now().nanoseconds/1e9-start<duration:
                r.spin(.05)
                if time.monotonic()>deadline:raise RuntimeError('Physics stopped advancing')
                row=r.rows[-1]
                elapsed=r.node.get_clock().now().nanoseconds/1e9-start
                if bias and elapsed-previous_bias>=1.:
                    east_bias=min(1.,elapsed/duration)*bias;previous_bias=elapsed
                    f=client.call_async(SetParameters.Request(parameters=[Parameter('east_bias',value=east_bias).to_parameter_msg()]))
                    r.wait(f.done,3,'GNSS slow bias update')
                    if not f.result().results[0].successful:raise RuntimeError('GNSS bias not applied')
                    bias_updates.append(dict(sim=r.node.get_clock().now().nanoseconds/1e9,east_bias_m=east_bias))
                if r.contacts or math.dist(row['truth'][:2],anchor)>5. or row['clearance_m']<1.:
                    raise RuntimeError('Independent evaluation safety boundary reached')
            rows=r.rows[begin:];errors=[math.dist(q['truth'][:2],q['ekf'][:2]) for q in rows]
            result['profiles'].append(dict(profile=name,settings=values,east_bias_ramp_target_m=bias,
                bias_updates=bias_updates,samples=len(rows),
                duration_sim_s=rows[-1]['sim']-rows[0]['sim'],
                reported_fix_types=sorted({q.get('GPS_RAW_INT',{}).get('fix_type',-1) for q in rows}),
                modes=sorted({q['fcu_state']['mode'] for q in rows}),last_mode=rows[-1]['fcu_state']['mode'],
                localization_rms_m=float(np.sqrt(np.mean(np.square(errors)))),localization_max_m=max(errors),
                original_target_truth_error_max_m=max(math.dist(q['truth'][:2],anchor) for q in rows),
                final_ekf_status=rows[-1].get('EKF_STATUS_REPORT'),
                max_abs_thrust_n=max(abs(q.get(k,0)) for q in rows for k in ('left_n','right_n')),
                collision_messages=len(r.contacts)))
            (r.run/'loiter_gnss_result.json').write_text(json.dumps(result,indent=2)+'\n')
        result['measurement_completed']=True
        result['note']='Native FCU behavior only. No RTK quality gate or automatic mode recovery added; preserving reported sensor quality is not proof of truth accuracy.'
    except Exception as error:
        result['error']=str(error);raise
    finally:
        result['parameters']=r.parameters
        result['parameter_attempts']=r.parameter_attempts
        (r.run/'loiter_gnss_result.json').write_text(json.dumps(result,indent=2)+'\n')
        (r.run/'loiter_targets.json').write_text(json.dumps(r.targets,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
        r.close();rclpy.try_shutdown()


if __name__=='__main__':main()
