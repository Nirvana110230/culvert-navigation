"""Run one immutable case manifest through one persistent registration CLI on Linux."""
import argparse
import itertools
import json
import os
from pathlib import Path
import platform
import select
import subprocess
import time

import numpy as np

from common import digest, error, read_json, write_json


PROTOCOL = dict(radius=12., budget_ms=500., good_translation_m=.15, good_rotation_deg=3.,
                ambiguity_rmse_m=.025, ambiguity_overlap=.08, distinct_translation_m=.75,
                distinct_rotation_deg=12., cold_repeats=2, hot_repeats=9, seeds=[41713, 68023, 93487])


def measure(path, keys):
    try:
        values = {a: b.strip() for a, b in (line.split(':', 1) for line in path.read_text().splitlines() if ':' in line)}
        return {k: values[k] for k in keys}
    except (OSError, KeyError):
        return {}


def resource(pid):
    proc = Path('/proc') / str(pid)
    stat = (proc / 'stat').read_text().rsplit(')', 1)[-1].split()
    status = measure(proc / 'status', ['VmRSS', 'VmHWM', 'Threads', 'Cpus_allowed_list'])
    return dict(cpu_s=(int(stat[11]) + int(stat[12])) / os.sysconf('SC_CLK_TCK'),
                rss_kib=int(status['VmRSS'].split()[0]), hwm_kib=int(status['VmHWM'].split()[0]),
                os_threads=int(status['Threads']), affinity=status['Cpus_allowed_list'])


class Engine:
    def __init__(self, binary, data, log, threads):
        self.data = data
        env = dict(os.environ, OMP_NUM_THREADS=str(threads), OMP_DYNAMIC='FALSE', OPENBLAS_NUM_THREADS='1')
        self.p = subprocess.Popen([str(binary)], env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=log, text=True, bufsize=1)

    def request(self, req):
        self.p.stdin.write(json.dumps(req, allow_nan=False) + '\n')
        self.p.stdin.flush()
        if not select.select([self.p.stdout], [], [], 120)[0]:
            raise TimeoutError(f'CLI exceeded 120 seconds for {req["op"]}')
        reply = self.p.stdout.readline()
        if not reply:
            raise RuntimeError(f'CLI closed stdout (exit={self.p.poll()})')
        return json.loads(reply)

    def close(self):
        self.p.stdin.close()
        try:
            self.p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.p.terminate()
            self.p.wait(timeout=5)
        self.p.stdout.close()

    def load(self, case):
        req = dict(op='load', path=str(self.data / case['map']), id=Path(case['map']).stem,
                   sha256=case['map_sha256'], version='r2_v1', frame=case['map_frame'], units='m',
                   origin='FAST-LIVO2 raw W' if case['split'] == 'history' else 'synthetic local origin',
                   gravity='FAST-LIVO2 raw W before export rotation' if case['split'] == 'history' else '+Z up')
        t = time.monotonic()
        reply = self.request(req)
        if not reply.get('ok'):
            raise ValueError(f'failed to load {case["map"]}: {reply}')
        self.generation = reply['generation']
        return dict(reply=reply, wall_ms=(time.monotonic() - t)*1000, resource=resource(self.p.pid))

    def match(self, c, backend, threads, P):
        before = resource(self.p.pid)
        t = time.monotonic()
        replies = []
        for idx, prior in enumerate(c['candidates']):
            req = dict(op='match', backend=backend, threads=threads, radius=P['radius'],
                       budget_ms=P['budget_ms'], generation=self.generation, initial=prior,
                       local=c['local'], source_frame=c['source_frame'], scan_id=str(c['scan_id']),
                       stamp_ns=c['stamp_ns'], scan_path=str(self.data / c['scan']))
            replies.append(self.request(req))
        elapsed = (time.monotonic() - t)*1000
        after = resource(self.p.pid)
        choice, accepted, reason = select_candidate(replies, elapsed, P)
        reply = replies[choice]
        estimate = reply.get('T_target_source')
        return dict(case=c['id'], backend=backend, threads=threads, selected=choice,
                    accepted=accepted, reason=reason, wall_ms=elapsed, after=after,
                    cpu_ms=(after['cpu_s'] - before['cpu_s'])*1000, replies=replies,
                    error=error(estimate, c['truth']) if estimate is not None and c['truth'] is not None else None,
                    reference_delta=error(estimate, c['reference']) if estimate is not None and c['reference'] else None)


def select_candidate(replies, elapsed, P):
    qualified = sorted((i for i, r in enumerate(replies) if r.get('accepted') and
                        r.get('rmse') is not None), key=lambda i: replies[i]['rmse'])
    if not qualified:
        return 0, False, replies[0].get('reason', 'no_qualified_candidate')
    i = qualified[0]
    if elapsed > P['budget_ms']:
        return i, False, 'group_budget'
    best = replies[i]
    for j in qualified[1:]:
        other = replies[j]
        delta = error(best['T_target_source'], other['T_target_source'])
        if ((delta['translation_m'] >= P['distinct_translation_m'] or
             delta['rotation_deg'] >= P['distinct_rotation_deg']) and
                abs(best['rmse'] - other['rmse']) <= P['ambiguity_rmse_m'] and
                abs(best['inlier_ratio'] - other['inlier_ratio']) <= P['ambiguity_overlap']):
            return i, False, 'ambiguous_multiple_candidates'
    return i, True, best.get('reason', 'accepted')


def write_row(stream, row):
    stream.write(json.dumps(row, allow_nan=False, separators=(',', ':')) + '\n')
    stream.flush()


def run_cases(binary, data, cases, output, splits, params):
    with (output / 'raw.jsonl').open('x', encoding='utf-8') as stream, (output / 'cli.stderr').open('x') as log:
        for backend in ['icp', 'gicp']:
            engine = Engine(binary, data, log, 1)
            try:
                tests = sorted((c for c in cases if c['split'] in splits and c['split'] != 'scale'), key=lambda c: c['map'])
                for _, group in itertools.groupby(tests, key=lambda c: c['map']):
                    group = list(group)
                    load = engine.load(group[0])
                    for c in group:
                        row = engine.match(c, backend, 1, params)
                        write_row(stream, dict(row, load=load, kind='accuracy'))
            finally:
                engine.close()
        if 'scale' not in splits:
            return
        for c in [c for c in cases if c['split'] == 'scale']:
            for backend, threads in itertools.product(['icp', 'gicp'], [1, 2, 4]):
                for trial in range(params['cold_repeats']):
                    # New process: cold application/submap/covariance cache; OS file cache left untouched.
                    engine = Engine(binary, data, log, threads)
                    try:
                        load = engine.load(c)
                        for rep in range(params['hot_repeats'] + 1):
                            row = engine.match(c, backend, threads, params)
                            write_row(stream, dict(row, kind='performance', trial=trial, rep=rep, load=load))
                        if c['tiles'] >= 5 and trial == 0:
                            # Visit A->B->C->A in a 2-entry LRU. All tiles contain the same local scene.
                            for step, offset in enumerate([0, 32., 64., 0]):
                                p = np.array(c['initial'])
                                p[0, 3] += offset
                                shifted = dict(c, candidates=[p.tolist()], truth=p.tolist())
                                row = engine.match(shifted, backend, threads, params)
                                write_row(stream, dict(row, kind='eviction', step=step, load=load))
                    finally:
                        engine.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cli', type=Path, required=True)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--splits', default='debug,tune,r1,holdout,history,scale')
    parser.add_argument('--freeze', type=Path, help='Require an existing immutable freeze from a preceding stage')
    a = parser.parse_args()
    data, binary = a.data.resolve(), a.cli.resolve()
    frozen = dict(protocol=PROTOCOL, binary_sha256=digest(binary), dataset_sha256=digest(data / 'dataset.json'),
                  runner_sha256=digest(__file__))
    if a.freeze and read_json(a.freeze) != frozen:
        raise ValueError('changed binary, dataset, runner or protocol since freeze')
    a.out.mkdir(parents=True, exist_ok=False)
    write_json(a.out / 'freeze.json', frozen)
    write_json(a.out / 'environment.json', dict(uname=list(platform.uname()), utc_start=time.strftime('%FT%TZ', time.gmtime()),
                                               affinity=sorted(os.sched_getaffinity(0)), cli=str(binary),
                                               dataset=str(data), splits=a.splits))
    cases = read_json(data / 'dataset.json')['cases']
    run_cases(binary, data, cases, a.out, a.splits.split(','), PROTOCOL)
    print(f'completed {a.out}')


if __name__ == '__main__':
    main()
