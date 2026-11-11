#!/usr/bin/env python3
"""Operator Control Station relay: vessel reports (UDP, JSON) -> RoboCommand (MQTT, protobuf).

Runs on the OCS computer, not on the vessel and without ROS. The 2026 handbook
(section 3.4) allows only the OCS to connect to RoboCommand. Needs:
    python3 -m pip install paho-mqtt protobuf
    git clone https://github.com/robonation/robocommand   (for RobotX_2026/gen/python)

Example:
    python3 scripts/ocs_relay.py --gen ~/robocommand/RobotX_2026/gen/python \\
        --team-id <ID from RoboNation> --vehicle-id USV1 --broker 192.168.65.2

--declare-task1 core sends one RunDeclaration for this vehicle at start-up.
Received RxCourse and RxCommand (RunStart) messages are printed.
"""
import argparse
import json
import socket
import sys
import time

ROOT = 'robocommand/robotx'
BODIES = dict(heartbeat='heartbeat', safe_passage='safe_passage')
TIERS = dict(none='TIER_NONE', core='TIER_CORE', advanced='TIER_ADVANCED', disruptive='TIER_DISRUPTIVE')


def load_schema(gen):
    """Import the checked-in generated modules from robocommand/RobotX_2026/gen/python."""
    if gen and gen not in sys.path:
        sys.path.insert(0, gen)
    from robotx import rx_commands_pb2, rx_course_pb2, rx_reports_pb2, rx_requests_pb2
    return dict(reports=rx_reports_pb2, requests=rx_requests_pb2, commands=rx_commands_pb2,
                course=rx_course_pb2)


def stamp(message, now):
    message.sent_at.seconds = int(now)
    message.sent_at.nanos = int((now-int(now))*1e9)


def build_report(schema, team_id, vehicle_id, seq, kind, body, now):
    """RxReport for one vessel datagram; raises ValueError on anything unexpected."""
    from google.protobuf import json_format
    if kind not in BODIES:
        raise ValueError(f'Unknown report kind {kind!r}')
    report = schema['reports'].RxReport(team_id=team_id, vehicle_id=vehicle_id, seq=seq)
    stamp(report, now)
    try:
        json_format.ParseDict(body, getattr(report, BODIES[kind]))
    except json_format.ParseError as error:
        raise ValueError(str(error))
    return report


def build_declaration(schema, team_id, seq, vehicle_ids, task1, now, task2='none', task3='none', task4='none'):
    request = schema['requests'].RxRequest(team_id=team_id, seq=seq)
    stamp(request, now)
    declaration = request.run_declaration
    declaration.vehicle_ids.extend(vehicle_ids)
    for field, tier in (('task1_tier', task1), ('task2_tier', task2), ('task3_tier', task3), ('task4_tier', task4)):
        setattr(declaration, field, TIERS[tier])
    return request


def report_topic(team_id, vehicle_id):
    return f'{ROOT}/{team_id}/{vehicle_id}/report'


class Relay:
    """Sequence numbers and the 5 messages-per-second limit for one vehicle."""

    def __init__(self, schema, team_id, vehicle_id, publish, max_rate=5.0, build=None):
        self.schema, self.team_id, self.vehicle_id, self.publish = schema, team_id, vehicle_id, publish
        self.build = build or build_report
        self.seq, self.sent, self.max_rate, self.dropped = 0, [], max_rate, 0

    def handle(self, datagram, now):
        """Forward one vessel datagram; returns the RxReport or None if refused."""
        try:
            packet = json.loads(datagram)
            if packet.get('vehicle_id') != self.vehicle_id:
                raise ValueError(f"vehicle_id {packet.get('vehicle_id')!r} is not {self.vehicle_id!r}")
            self.sent = [t for t in self.sent if now-t < 1.0]
            if len(self.sent) >= self.max_rate:
                self.dropped += 1
                return None
            report = self.build(self.schema, self.team_id, self.vehicle_id, self.seq+1,
                                packet.get('kind'), packet.get('body'), now)
        except (ValueError, TypeError, AttributeError) as error:
            print(f'Refused datagram: {error}', file=sys.stderr)
            return None
        self.seq += 1
        self.sent.append(now)
        self.publish(report_topic(self.team_id, self.vehicle_id), report.SerializeToString())
        return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--gen', required=True, help='path to robocommand/RobotX_2026/gen/python')
    parser.add_argument('--team-id', required=True)
    parser.add_argument('--vehicle-id', default='USV1')
    parser.add_argument('--broker', required=True, help='RoboCommand address, e.g. 192.168.65.2 on Course Alpha')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--listen', default='0.0.0.0:47800', help='UDP address for vessel datagrams')
    parser.add_argument('--declare-task1', choices=sorted(TIERS), help='send one RunDeclaration at start-up')
    args = parser.parse_args(argv)
    import paho.mqtt.client as mqtt
    schema = load_schema(args.gen)
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:  # paho-mqtt 1.x
        client = mqtt.Client()

    def on_message(_client, _userdata, message):
        kind = schema['course'].RxCourse if message.topic.endswith('/course') else schema['commands'].RxCommand
        decoded = kind()
        decoded.ParseFromString(message.payload)
        print(f'<- {message.topic}\n{decoded}', flush=True)

    client.on_message = on_message
    client.connect(args.broker, args.port, keepalive=30)
    client.subscribe([(f'{ROOT}/course', 1), (f'{ROOT}/{args.team_id}/command', 1)])
    client.loop_start()
    if args.declare_task1:
        request = build_declaration(schema, args.team_id, 1, [args.vehicle_id], args.declare_task1, time.time())
        client.publish(f'{ROOT}/{args.team_id}/request', request.SerializeToString(), qos=1)
        print(f'-> RunDeclaration task1={args.declare_task1} vehicles={[args.vehicle_id]}', flush=True)
    relay = Relay(schema, args.team_id, args.vehicle_id, lambda topic, payload: client.publish(topic, payload))
    host, port = args.listen.rsplit(':', 1)
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((host, int(port)))
    server.settimeout(1.0)
    last, shown = time.time(), 0
    print(f'Relaying UDP {args.listen} -> mqtt://{args.broker}:{args.port} as {args.team_id}/{args.vehicle_id}')
    try:
        while True:
            try:
                relay.handle(server.recvfrom(65535)[0], time.time())
                last = time.time()
            except socket.timeout:
                pass
            if time.time()-shown >= 5.0:
                shown = time.time()
                silent = time.time()-last
                print(f'sent {relay.seq}, dropped {relay.dropped}'
                      + (f', NO VESSEL DATA for {silent:.0f} s: no heartbeat reaches RoboCommand' if silent > 2 else ''),
                      flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()
    return 0


if __name__ == '__main__':
    sys.exit(main())
