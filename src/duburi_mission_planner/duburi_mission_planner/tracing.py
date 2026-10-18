"""Mission tracing: one JSON file per mission with everything needed to audit or replay it."""
from __future__ import annotations

import json
import os
import time
import uuid
from typing import Any, Optional


def new_mission_id() -> str:
    return time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6]


class MissionTrace:
    def __init__(self, prompt: str, provider: dict, out_dir: Optional[str] = None, mission_id: Optional[str] = None):
        self.mission_id = mission_id or new_mission_id()
        self.out_dir = out_dir
        self.t0 = time.time()
        self.data: dict[str, Any] = {
            'mission_id': self.mission_id,
            'prompt': prompt,
            'provider': provider,
            'started_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'llm_calls': [],
            'plans': [],
            'validations': [],
            'tool_calls': [],
            'replans': [],
            'events': [],
            'outcome': None,
        }

    def _t(self):
        return round(time.time() - self.t0, 3)

    def event(self, kind: str, **data):
        self.data['events'].append({'t': self._t(), 'kind': kind, **data})

    def llm_calls(self, calls: list[dict]):
        for c in calls:
            c = dict(c)
            c['t'] = round(c.pop('t_wall', time.time()) - self.t0, 3)
            self.data['llm_calls'].append(c)

    def plan(self, plan_index: int, kind: str, plan: Optional[dict], status: str, reason: str = ''):
        self.data['plans'].append({'t': self._t(), 'index': plan_index, 'kind': kind, 'status': status,
                                   'reason': reason, 'plan': plan})

    def validation(self, plan_index: int, report: dict):
        self.data['validations'].append({'t': self._t(), 'plan_index': plan_index, **report})

    def tool_call(self, record: dict):
        self.data['tool_calls'].append({'t': self._t(), **record})

    def finish(self, outcome: dict) -> dict:
        self.data['outcome'] = outcome
        self.data['wall_s'] = self._t()
        self.data['finished_at'] = time.strftime('%Y-%m-%dT%H:%M:%S%z')
        return self.data

    def save(self) -> Optional[str]:
        if not self.out_dir:
            return None
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(self.out_dir, f'trace_{self.mission_id}.json')
        with open(path, 'w') as f:
            json.dump(self.data, f, indent=2, default=str)
        return path
