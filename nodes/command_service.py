#!/usr/bin/env python3
# Written 2026-10-29: understand a spoken or typed robot command with Claude. New file.
"""Turn a spoken/typed command ("go to Hemant, take something from him, then deliver it to Rahul's
office") into a checked multi-stop trip plan that nodes/fleet.py can run.

Runs in its own venv (the anthropic SDK can't be pip-installed into the system python ROS uses):

    ~/.venvs/llm/bin/python nodes/command_service.py        # 127.0.0.1:8071  (run/command.sh on)

Claude only *understands*: it returns structured JSON, constrained to the real place ids of the current
map, and the dashboard shows the plan for a human CONFIRM before the robot moves. Unclear commands
come back as a question instead of a guess.

    POST /parse  {"text": "...", "places": [{"id", "name"}], "requester": {"name", "place_id"}}
         -> {"ok": true, "plan": {understood, legs: [{action, place_id, note}], item, question, summary}}
    GET  /health -> {"ok", "key_configured", "model"}

API key: ANTHROPIC_API_KEY, else ~/.campus_bot/anthropic.key (0600, outside the repo; never commit it).
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import anthropic

HOST, PORT = '127.0.0.1', int(os.environ.get('COMMAND_PORT', '8071'))
KEY_FILE = os.path.expanduser('~/.campus_bot/anthropic.key')
MODEL = os.environ.get('COMMAND_MODEL', 'claude-opus-5-5')
# A short extraction, not a hard reasoning task: low effort keeps it to a few seconds. (Thinking can't be
# turned off on Claude Opus 5.5; effort is the control.)
EFFORT = os.environ.get('COMMAND_EFFORT', 'low')
MAX_LEGS = 6
MAX_TEXT = 500

SYSTEM = """You turn spoken or typed commands for an office delivery robot into a trip plan.

The robot drives between named places on one office floor and carries small items (under 5 kg) in an
open crate. At each stop it does one of:
- "pickup": drive there and wait until someone loads the item into the crate.
- "deliver": drive there and wait until the recipient takes the item.
- "go": just drive there (no item handling), e.g. "come to the meeting room".
It cannot open doors, use lifts, buy things, or do anything except carry items between places.

Rules:
- Use only place ids from the list you are given. People are reached at their own desk or office:
  "Hemant" means the place named like "Hemant's Desk"; "Rahul's office" means "Rahul Sir Office".
- "take / get / collect / pick up X from A" is a pickup at A. "give / deliver / drop / bring / take X to
  B" is a deliver at B. Taking something from someone and bringing it to someone else is a pickup
  followed by a deliver.
- "me", "my desk", "here" mean the requester's place when it is given; if it is not given, ask.
- The text comes from speech recognition: names may be misspelled or sound-alike ("Babneet" for
  "Babneek", "Rahul's office" for "Rahul Sir Office"). Map them to the closest place when you are
  confident.
- If a name matches several places or none, if a needed place is missing, or if the command is not a
  task this robot can do, set understood to false, legs to [], and ask one short clarifying question.
- summary: one short sentence the robot can read back, using the place names, e.g. "I'll collect the
  item from Hemant's Desk and deliver it to Rahul Sir Office."
- note: a few words for that stop, e.g. "collect the item from Hemant". item: what is carried, or
  "something" when not said. question: empty string when understood is true."""


def schema(place_ids):
    return {
        'type': 'object',
        'properties': {
            'understood': {'type': 'boolean'},
            'legs': {'type': 'array', 'items': {
                'type': 'object',
                'properties': {
                    'action': {'type': 'string', 'enum': ['pickup', 'deliver', 'go']},
                    'place_id': {'type': 'string', 'enum': place_ids},
                    'note': {'type': 'string'},
                },
                'required': ['action', 'place_id', 'note'],
                'additionalProperties': False}},
            'item': {'type': 'string'},
            'question': {'type': 'string'},
            'summary': {'type': 'string'},
        },
        'required': ['understood', 'legs', 'item', 'question', 'summary'],
        'additionalProperties': False,
    }


def api_key():
    k = os.environ.get('ANTHROPIC_API_KEY', '').strip()
    if not k:
        try:
            with open(KEY_FILE) as f:
                k = f.read().strip()
        except OSError:
            k = ''
    return k


_client = {'key': None, 'c': None}
_lock = threading.Lock()


def client():
    """The SDK client, rebuilt if the key file changed (so adding the key needs no restart)."""
    k = api_key()
    if not k:
        return None
    with _lock:
        if _client['key'] != k:
            _client['c'] = anthropic.Anthropic(api_key=k, timeout=30.0, max_retries=1)
            _client['key'] = k
        return _client['c']


def check_plan(plan, places):
    """Server-side sanity on top of the schema: known places, sensible length, a question when unsure."""
    ids = {p['id'] for p in places}
    legs = plan.get('legs') or []
    if plan.get('understood'):
        if not legs:
            return 'the plan has no stops'
        if len(legs) > MAX_LEGS:
            return f'too many stops (max {MAX_LEGS})'
        for leg in legs:
            if leg.get('place_id') not in ids or leg.get('action') not in ('pickup', 'deliver', 'go'):
                return f'bad stop {leg}'
        for a, b in zip(legs, legs[1:]):
            if a['place_id'] == b['place_id'] and a['action'] == b['action']:
                return 'the same stop twice in a row'
    elif not (plan.get('question') or '').strip():
        plan['question'] = 'Sorry, I didn\'t understand. Which place should I go to, and what should I do there?'
    return None


def parse(text, places, requester=None):
    t0 = time.time()
    text = (text or '').strip()[:MAX_TEXT]
    if not text:
        return {'ok': False, 'error': 'empty command'}
    if not places:
        return {'ok': False, 'error': 'no places are pinned on this map yet'}
    c = client()
    if c is None:
        return {'ok': False, 'error': f'no Anthropic API key: put it in {KEY_FILE}'}
    places = [{'id': str(p['id']), 'name': str(p['name'])} for p in places]
    req = requester or {}
    lines = '\n'.join(f'- {p["id"]}: {p["name"]}' for p in places)
    who = req.get('name') or 'unknown'
    here = next((p['name'] for p in places if p['id'] == req.get('place_id')), None)
    user = (f'Places on this floor (id: name):\n{lines}\n\n'
            f'Requester: {who}; requester\'s place: {here or "not given"}\n\n'
            f'Command: {text}')
    try:
        resp = c.beta.messages.create(
            model=MODEL,
            max_tokens=4096,
            # On a safety-classifier decline the API re-runs the request on Anthropic's recommended
            # fallback model inside the same call (routed by refusal category).
            betas=['server-side-fallback-2026-07-01'],
            fallbacks='default',
            output_config={'effort': EFFORT,
                           'format': {'type': 'json_schema', 'schema': schema([p['id'] for p in places])}},
            system=SYSTEM,
            messages=[{'role': 'user', 'content': user}],
        )
    except anthropic.AuthenticationError:
        return {'ok': False, 'error': 'the Anthropic API key was rejected: check ~/.campus_bot/anthropic.key'}
    except anthropic.PermissionDeniedError:
        return {'ok': False, 'error': 'the API key may not use this model'}
    except anthropic.RateLimitError:
        return {'ok': False, 'error': 'too many commands right now: try again in a minute'}
    except anthropic.BadRequestError as e:
        return {'ok': False, 'error': f'request rejected: {e.message}'}
    except anthropic.APIStatusError as e:
        return {'ok': False, 'error': f'Claude API error {e.status_code}: try again'}
    except anthropic.APIConnectionError:
        return {'ok': False, 'error': 'no connection to Claude (is the robot online?)'}
    if resp.stop_reason == 'refusal':
        return {'ok': False, 'error': 'that command was declined'}
    if resp.stop_reason == 'max_tokens':
        return {'ok': False, 'error': 'the answer was cut off: try a shorter command'}
    text_out = next((b.text for b in resp.content if b.type == 'text'), '')
    try:
        plan = json.loads(text_out)
    except ValueError:
        return {'ok': False, 'error': 'could not read the plan: try again'}
    bad = check_plan(plan, places)
    if bad:
        return {'ok': False, 'error': f'the plan didn\'t make sense ({bad}): try rephrasing'}
    names = {p['id']: p['name'] for p in places}
    for leg in plan['legs']:
        leg['place_name'] = names[leg['place_id']]
    return {'ok': True, 'plan': plan, 'model': resp.model, 'ms': int((time.time() - t0) * 1000),
            'request_id': getattr(resp, '_request_id', None)}


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, obj):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path == '/health':
            return self._send(200, {'ok': True, 'key_configured': bool(api_key()), 'model': MODEL,
                                    'effort': EFFORT})
        self._send(404, {'ok': False, 'error': 'not found'})

    def do_POST(self):
        if self.path != '/parse':
            return self._send(404, {'ok': False, 'error': 'not found'})
        try:
            n = int(self.headers.get('Content-Length', 0) or 0)
            body = json.loads(self.rfile.read(min(n, 1 << 20)) or b'{}')
            if not isinstance(body, dict):
                raise ValueError
        except ValueError:
            return self._send(400, {'ok': False, 'error': 'bad JSON body'})
        r = parse(body.get('text'), body.get('places') or [], body.get('requester'))
        print(time.strftime('%H:%M:%S'), 'parse', 'ok' if r.get('ok') else 'FAIL',
              json.dumps({'text': (body.get('text') or '')[:120],
                          'summary': (r.get('plan') or {}).get('summary'), 'error': r.get('error'),
                          'ms': r.get('ms'), 'request_id': r.get('request_id')}), flush=True)
        self._send(200, r)

    def log_message(self, *a):
        pass


def main():
    srv = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f'command service on http://{HOST}:{PORT}  (model {MODEL}, effort {EFFORT}; '
          f'API key {"found" if api_key() else "MISSING: " + KEY_FILE})', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    sys.exit(main())
