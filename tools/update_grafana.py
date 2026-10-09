#!/usr/bin/env python3
"""Update a fetched dashboard without replacing unrelated panels.

Usage: python3 tools/update_grafana.py INPUT OUTPUT
INPUT is GET /api/dashboards/uid/birdburner-controller's JSON response.
OUTPUT is the body for POST /api/dashboards/db. No credentials are stored here.
"""
import copy
import json
import sys


def update(source):
    dashboard = copy.deepcopy(source['dashboard'])
    panels = dashboard['panels']
    by_title = {p['title']: p for p in panels}
    next_id = max(p['id'] for p in panels) + 1
    datasource = copy.deepcopy(by_title['Temperature & Control Curves']['datasource'])

    # Preserve the raw control measurements; a display moving average is not
    # the firmware's derivative low-pass filter.
    temperature = by_title['Temperature & Control Curves']
    temperature['description'] = (
        'Unfiltered control-window temperature, target, absolute software cutoff, '
        'and target + 1 C PID output guard (new firmware). '
        'Tf filters only the derivative term, not this measurement or safety checks.'
    )
    temperature['fieldConfig']['defaults']['custom']['lineInterpolation'] = 'linear'
    if not any(t['legendFormat'] == 'PID heat guard (+1 C)' for t in temperature['targets']):
        target = copy.deepcopy(temperature['targets'][0])
        target.update(refId='D', expr='birdburner_target_temperature_celsius + 1', legendFormat='PID heat guard (+1 C)')
        temperature['targets'].append(target)
        temperature['fieldConfig']['overrides'].append({
            'matcher': {'id': 'byName', 'options': 'PID heat guard (+1 C)'},
            'properties': [
                {'id': 'color', 'value': {'fixedColor': 'orange', 'mode': 'fixed'}},
                {'id': 'custom.lineStyle', 'value': {'dash': [4, 4], 'fill': 'dash'}},
            ],
        })

    specs = [
        ('PID Kp', 'PID Kp History', 'birdburner_pid_kp', 'Kp', 'none', 3,
         'Proportional gain, output %/C. Factory default 20.'),
        ('PID Ki', 'PID Ki History', 'birdburner_pid_ki', 'Ki', 'none', 3,
         'Integral gain, output %/(C*s). Factory default 0.02.'),
        ('PID Kd', 'PID Kd History', 'birdburner_pid_kd', 'Kd', 'none', 3,
         'Derivative gain on measurement, output %*s/C. Factory default 120.'),
        ('Derivative Filter Tf', 'Derivative Filter Tf History', 'birdburner_pid_d_filter_seconds', 'Tf (s)', 's', 3,
         'Derivative first-order low-pass time constant (seconds). Default 30, range 0-300; 0 = off. '
         'alpha = dt/(Tf+dt); at dt=5 s and Tf=30 s, alpha=0.142857. '
         'Not a temperature moving average.'),
    ]
    if not any(p['title'] == 'PID Kp' for p in panels):
        for panel in panels:
            if panel['gridPos']['y'] >= 8:
                panel['gridPos']['y'] += 4
    history_y = max(p['gridPos']['y'] + p['gridPos']['h'] for p in panels)

    for i, (title, history_title, expr, legend, unit, decimals, description) in enumerate(specs):
        target = {
            'datasource': copy.deepcopy(datasource), 'editorMode': 'code', 'expr': expr,
            'instant': True, 'range': False, 'legendFormat': legend, 'refId': 'A',
        }
        if title not in by_title:
            stat = copy.deepcopy(by_title['Uptime'])
            stat.update(id=next_id, title=title, description=description + ' Requires PID/filter firmware metrics; missing is not zero.',
                        gridPos={'h': 4, 'w': 6, 'x': i * 6, 'y': 8}, targets=[target])
            stat['fieldConfig'] = {'defaults': {
                'unit': unit, 'decimals': decimals, 'noValue': 'No metric',
                'color': {'mode': 'fixed', 'fixedColor': 'light-blue'},
                'thresholds': {'mode': 'absolute', 'steps': [{'color': 'green', 'value': None}]},
            }, 'overrides': []}
            panels.append(stat)
            next_id += 1
        if history_title not in by_title:
            history = copy.deepcopy(by_title['Temperature & Control Curves'])
            target = copy.deepcopy(target)
            target.update(instant=False, range=True)
            history.update(id=next_id, title=history_title, description=description,
                           gridPos={'h': 8, 'w': 6, 'x': i * 6, 'y': history_y}, targets=[target])
            history['fieldConfig']['overrides'] = []
            defaults = history['fieldConfig']['defaults']
            defaults.update(unit=unit, decimals=decimals, min=0)
            defaults['custom'].update(lineInterpolation='stepAfter', fillOpacity=0)
            history['options']['tooltip']['hideZeros'] = False
            panels.append(history)
            next_id += 1

    recovery = by_title['Probe Power Cycles & Recoveries']
    if not any(t.get('expr') == 'birdburner_heater_output_held' for t in recovery['targets']):
        target = copy.deepcopy(recovery['targets'][-1])
        target.update(refId='E', expr='birdburner_heater_output_held', legendFormat='Heater output held (0/1)')
        recovery['targets'].append(target)
    recovery['description'] = (
        'Power-cycle/recovery increases per 5 min; failed-window count and 0/1 recovery/heat-hold states. '
        'Heat hold is bounded, not a sensor-success signal.'
    )
    dashboard['time'] = {'from': 'now-1h', 'to': 'now'}
    dashboard['refresh'] = '5s'
    panels.sort(key=lambda p: (p['gridPos']['y'], p['gridPos']['x']))
    return {
        'dashboard': dashboard, 'folderUid': source.get('meta', {}).get('folderUid') or '',
        # Version checking prevents silently clobbering a simultaneous UI edit.
        'overwrite': False,
        'message': 'Add PID gains, derivative Tf and histories; preserve raw temperature and show +1 C heat guard.',
    }


if __name__ == '__main__':
    with open(sys.argv[1]) as f:
        source = json.load(f)
    result = update(source)
    with open(sys.argv[2], 'w') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
        f.write('\n')
