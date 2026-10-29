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


# ------------------------------------------------------------------ vision (the mast camera)
# Claude looks at one camera frame and answers in structured JSON. It takes seconds, so it only informs
# SLOW decisions (how long to wait when blocked, slowing down for something the lidar can't see, telling
# people what it found). Steering and collision safety stay with the lidar, bumper and watchdog.
# 2026-10-29: the camera now sits IN THE MIDDLE of the robot, low, looking forward over its own deck
# (it used to be described as a 1.25 m mast). The robot's own parts in view double as a ruler.
ROBOT = """You are the eyes of an indoor office delivery robot. The image comes from a forward-facing camera
mounted in the MIDDLE of the robot, low (roughly 25-40 cm above the floor), looking ahead and slightly down.

Parts of the robot itself are in the picture. They are never obstacles:
- the black glossy surface across the bottom of the image is the robot's own top deck; its far edge is the
  front of the robot;
- grey vertical aluminium bars at the left and right edges are the robot's own corner columns (its sides);
- anything lying on that black deck (labels, a blue box, cables on the deck) is on the robot.
Use them as a ruler: the robot is 44 cm long and 38 cm wide (the two columns are its width), with an open
crate on a platform about 50 cm high (crate top roughly 0.8 m). To pass between things it needs a gap of
about 50 cm. Top speed about 0.8 m/s.

It senses obstacles with a 2D lidar that scans ONE horizontal plane 21.5 cm above the floor, so the lidar
can MISS: objects lower than about 20 cm (bags, cables, shoes, chair bases and casters, low boxes, cups),
things above that plane that stick out (table tops, open drawers, shelf edges, chair seats and arms between
0.25 and 0.8 m), glass and mirrors, and spills.

Legroom (always estimate, in metres, from the robot's front edge): clear_ahead_m = free floor straight
ahead before anything the robot would touch, including things the lidar misses (use 5 if open for 5 m+);
path_width_m = the narrowest width of free floor along the next ~1.5 m of its path."""

ROOM = {'clear_ahead_m': {'type': 'number'}, 'path_width_m': {'type': 'number'}}

VISION = {
    'hazard': {
        'prompt': ROBOT + """

The robot is driving forward right now. Report only hazards IN ITS PATH (the floor area straight ahead,
roughly the middle of the image) that the lidar might miss. Do not report walls, desks or people standing
upright (the lidar sees those), or things clearly off to the side. If nothing qualifies, path_clear is true
and hazards is empty. Be conservative: a false alarm only slows the robot briefly, but don't invent hazards.""",
        'schema': {'type': 'object', 'properties': {
            'path_clear': {'type': 'boolean'},
            'hazards': {'type': 'array', 'items': {'type': 'object', 'properties': {
                'what': {'type': 'string'},
                'kind': {'type': 'string', 'enum': ['low_floor_object', 'overhang', 'glass_or_mirror',
                                                    'liquid_or_slippery', 'drop_or_step', 'other']},
                'where': {'type': 'string', 'enum': ['ahead', 'ahead_left', 'ahead_right']},
                'distance': {'type': 'string', 'enum': ['under_1m', '1_to_3m', 'over_3m']},
                'severity': {'type': 'string', 'enum': ['low', 'medium', 'high']}},
                'required': ['what', 'kind', 'where', 'distance', 'severity'], 'additionalProperties': False}},
            'summary': {'type': 'string'}, **ROOM},
            'required': ['path_clear', 'hazards', 'summary', *ROOM], 'additionalProperties': False},
    },
    'blocked': {
        'prompt': ROBOT + """

The robot's planner reports it cannot get through toward its destination. Look at what is in front of it
and say what is blocking it and whether that will clear by itself. people / a group usually move within a
minute (wait_short); furniture, boxes or a cart usually need someone to move them (wait_long, and say what to
move); a closed door means give_up. message: one short sentence for the office, e.g. "A chair is blocking
the corridor near the meeting room; could someone move it?".""",
        'schema': {'type': 'object', 'properties': {
            'cause': {'type': 'string', 'enum': ['person', 'group_of_people', 'furniture', 'object_on_floor',
                                                 'closed_door', 'narrow_gap', 'nothing_visible', 'unclear']},
            'will_clear_soon': {'type': 'boolean'},
            'advice': {'type': 'string', 'enum': ['wait_short', 'wait_long', 'give_up']},
            'what': {'type': 'string'},
            'message': {'type': 'string'}, **ROOM},
            'required': ['cause', 'will_clear_soon', 'advice', 'what', 'message', *ROOM],
            'additionalProperties': False},
    },
    'describe': {
        'prompt': ROBOT + """

Someone remotely operating the robot asks what it sees. Describe the scene in 2-3 short sentences: where it
seems to be, any people, and anything in its way, mentioning anything the lidar might miss.""",
        'schema': {'type': 'object', 'properties': {
            'description': {'type': 'string'},
            'people': {'type': 'integer'},
            'in_the_way': {'type': 'string'}, **ROOM},
            'required': ['description', 'people', 'in_the_way', *ROOM], 'additionalProperties': False},
    },
    'arrival': {
        'prompt': ROBOT + """

The robot has just arrived at a stop (given below) to hand over or collect an item. From the image: does
this look like that kind of place (a desk, an office, a meeting room), and is someone there to deal with the
robot? You cannot recognise who people are, and must not try; just say whether someone is present.
message: one short friendly sentence for the call page, e.g. "Arrived at the desk; nobody seems to be here
right now.".""",
        'schema': {'type': 'object', 'properties': {
            'looks_like_destination': {'type': 'string', 'enum': ['yes', 'no', 'unclear']},
            'person_present': {'type': 'string', 'enum': ['yes', 'no', 'unclear']},
            'message': {'type': 'string'}},
            'required': ['looks_like_destination', 'person_present', 'message'], 'additionalProperties': False},
    },
}
MAX_IMAGE_B64 = 3 << 20


def vision(mode, image_b64, context=None):
    t0 = time.time()
    spec = VISION.get(mode)
    if spec is None:
        return {'ok': False, 'error': f'unknown vision mode {mode}'}
    if not image_b64 or len(image_b64) > MAX_IMAGE_B64:
        return {'ok': False, 'error': 'no camera frame' if not image_b64 else 'frame too large'}
    c = client()
    if c is None:
        return {'ok': False, 'error': f'no Anthropic API key: put it in {KEY_FILE}'}
    ctx = '\n'.join(f'{k}: {v}' for k, v in (context or {}).items() if v not in (None, ''))
    content = [{'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/jpeg', 'data': image_b64}},
               {'type': 'text', 'text': (f'Context:\n{ctx}\n\n' if ctx else '') + 'Answer about this frame.'}]
    try:
        resp = c.beta.messages.create(
            model=MODEL, max_tokens=2048,
            betas=['server-side-fallback-2026-07-01'], fallbacks='default',
            output_config={'effort': EFFORT, 'format': {'type': 'json_schema', 'schema': spec['schema']}},
            system=spec['prompt'],
            messages=[{'role': 'user', 'content': content}],
        )
    except anthropic.AuthenticationError:
        return {'ok': False, 'error': 'the Anthropic API key was rejected'}
    except anthropic.RateLimitError:
        return {'ok': False, 'error': 'rate limited'}
    except anthropic.BadRequestError as e:
        return {'ok': False, 'error': f'request rejected: {e.message}'}
    except anthropic.APIStatusError as e:
        return {'ok': False, 'error': f'Claude API error {e.status_code}'}
    except anthropic.APIConnectionError:
        return {'ok': False, 'error': 'no connection to Claude'}
    if resp.stop_reason in ('refusal', 'max_tokens'):
        return {'ok': False, 'error': f'no answer ({resp.stop_reason})'}
    try:
        result = json.loads(next((b.text for b in resp.content if b.type == 'text'), ''))
    except ValueError:
        return {'ok': False, 'error': 'unreadable answer'}
    u = resp.usage
    return {'ok': True, 'mode': mode, 'result': result, 'ms': int((time.time() - t0) * 1000),
            'usage': {'in': u.input_tokens, 'out': u.output_tokens}, 'model': resp.model}


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
        if self.path not in ('/parse', '/vision'):
            return self._send(404, {'ok': False, 'error': 'not found'})
        try:
            n = int(self.headers.get('Content-Length', 0) or 0)
            body = json.loads(self.rfile.read(min(n, MAX_IMAGE_B64 + (1 << 16))) or b'{}')
            if not isinstance(body, dict):
                raise ValueError
        except ValueError:
            return self._send(400, {'ok': False, 'error': 'bad JSON body'})
        if self.path == '/vision':
            r = vision(str(body.get('mode', '')), body.get('image_b64'), body.get('context'))
            res = r.get('result') or {}
            print(time.strftime('%H:%M:%S'), 'vision', body.get('mode'), 'ok' if r.get('ok') else 'FAIL',
                  json.dumps({'ms': r.get('ms'), 'usage': r.get('usage'), 'error': r.get('error'),
                              'gist': res.get('summary') or res.get('message') or res.get('description')}),
                  flush=True)
            return self._send(200, r)
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
