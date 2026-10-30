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
- Places also have a NUMBER: the label on its pin on the dashboard map (pin 1, 2, 3, ...). "point 3",
  "pin 3", "number three", "location 3", "spot 3", "go to 3", "deliver it to five" all mean the place with
  that number (speech recognition may write the number as a word or a digit, or as "to" for "two" /
  "for" for "four" only when nothing else fits). A number with no place behind it: ask.
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
    lines = '\n'.join(f'- pin {i}: id {p["id"]}: {p["name"]}' for i, p in enumerate(places, 1))
    who = req.get('name') or 'unknown'
    here = next((p['name'] for p in places if p['id'] == req.get('place_id')), None)
    user = (f'Places on this floor (pin number on the map, id, name):\n{lines}\n\n'
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
        leg['pin'] = [p['id'] for p in places].index(leg['place_id']) + 1
    return {'ok': True, 'plan': plan, 'model': resp.model, 'ms': int((time.time() - t0) * 1000),
            'request_id': getattr(resp, '_request_id', None)}


# ------------------------------------------------------------------ vision (the mast camera)
# Claude looks at one camera frame and answers in structured JSON. It takes seconds, so it only informs
# SLOW decisions (how long to wait when blocked, slowing down for something the lidar can't see, telling
# people what it found). Steering and collision safety stay with the lidar, bumper and watchdog.
# 2026-10-29: the camera now sits IN THE MIDDLE of the robot, low, looking forward over its own deck
# (it used to be described as a 1.25 m mast). The robot's own parts in view double as a ruler.
ROBOT = """You are the eyes of an indoor office delivery robot. The image comes from a forward-facing camera
mounted near the MIDDLE of the robot: lens 39 cm above the floor, 7.5 cm behind the lidar (so about 30 cm
behind the robot's front edge), looking ahead and about 25 degrees down: it sees the floor from about 0.7 m
to 2 m ahead (the top edge of the image is ~2 m away at floor level).

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

The robot is driving forward right now. This robot can NEVER pass under a table or desk (its crate platform is too tall), so a table or desk in
the path is always a hazard, even when the floor under it looks open. Most hazards here are table and chair
LEGS and chair star-bases / wheels: thin or low, so the 2D lidar (21 cm up) sees them badly or not at all.
Report only hazards IN ITS PATH (the floor area straight ahead,
roughly the middle of the image) that the lidar might miss. Do not report walls, desks or people standing
upright (the lidar sees those), or things clearly off to the side. If nothing qualifies, path_clear is true
and hazards is empty. Be conservative: a false alarm makes the robot slow down and steer around the spot,
so don't invent hazards.

Where each hazard is in THIS image, as fractions of the image (0,0 = top-left corner, 1,1 = bottom-right):
x_left / x_right = its left and right edges; y_floor = the height in the image where its base touches the
floor (for an overhang: where the floor directly under its front edge is; for a drop or step: its near edge).
The robot uses these to mark the spot on its map and plan around it, so be as exact as you can.""",
        'schema': {'type': 'object', 'properties': {
            'path_clear': {'type': 'boolean'},
            'hazards': {'type': 'array', 'items': {'type': 'object', 'properties': {
                'what': {'type': 'string'},
                'kind': {'type': 'string', 'enum': ['table_or_chair_leg', 'table_or_desk', 'low_floor_object', 'overhang', 'glass_or_mirror',
                                                    'liquid_or_slippery', 'drop_or_step', 'other']},
                'where': {'type': 'string', 'enum': ['ahead', 'ahead_left', 'ahead_right']},
                'distance': {'type': 'string', 'enum': ['under_1m', '1_to_3m', 'over_3m']},
                'severity': {'type': 'string', 'enum': ['low', 'medium', 'high']},
                'x_left': {'type': 'number'}, 'x_right': {'type': 'number'}, 'y_floor': {'type': 'number'}},
                'required': ['what', 'kind', 'where', 'distance', 'severity', 'x_left', 'x_right', 'y_floor'],
                'additionalProperties': False}},
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

# ---- the robot is obstructed: look through every view at once and plan a short way around (2026-10-30)
VISION['maneuver'] = {
    'effort': 'medium', 'max_tokens': 8000,
    'prompt': ROBOT + """

The robot is OBSTRUCTED: something is close in front of it or it could not get through. Several views are
attached, each labelled:
- FRONT CAMERA: the camera described above (the floor 0.7-2 m ahead).
- HIGH CAMERA (if attached): a second camera higher up on the robot, also facing forward, wider view (it may
  show table edges and legs from above). Its picture is upright.
- LIDAR MAP: top-down, 1 px = 1 cm, the robot at the centre facing UP (the green rectangle is its exact
  outline, 44 x 38 cm), white dots = lidar returns 21 cm above the floor, red X = hazards the camera marked
  (things the lidar may miss), yellow arrow = straight-line direction to its destination, grid lines every
  0.5 m. Left in the map is the robot's left.

Plan at most 3 small moves that get it around the obstruction toward the destination. Moves:
back (metres, 0.05-0.5), forward (metres, 0.05-0.8), turn_left / turn_right (degrees, 5-120, on the spot
about its centre). Then normal navigation takes over again (then = resume): you only need to get it into
a clear position facing a free gap, not all the way. It can NEVER go under a table or desk; table and chair
legs and chair bases are the usual problem. A gap must be at least 50 cm wide. Every move is checked against
the lidar before it runs and stopped if it would touch something, so prefer moves through space the lidar
map shows empty, and keep 10 cm from red X marks. If there is no way through (a closed door, a blocked
corridor, people standing there), plan no moves and set then to wait (it will clear) or give_up.
obstruction: what is in the way, few words. reason: one sentence why these moves. message: one short
sentence for the office if it must wait or give up, otherwise empty.""",
    'schema': {'type': 'object', 'properties': {
        'obstruction': {'type': 'string'},
        'moves': {'type': 'array', 'items': {'type': 'object', 'properties': {
            'action': {'type': 'string', 'enum': ['back', 'forward', 'turn_left', 'turn_right']},
            'amount': {'type': 'number'}},
            'required': ['action', 'amount'], 'additionalProperties': False}},
        'then': {'type': 'string', 'enum': ['resume', 'wait', 'give_up']},
        'reason': {'type': 'string'}, 'message': {'type': 'string'}},
        'required': ['obstruction', 'moves', 'then', 'reason', 'message'], 'additionalProperties': False},
}


def vision(mode, image_b64, context=None, images=None):
    """images: optional [{'label', 'b64', 'type'}] for several views at once (maneuver); else image_b64."""
    t0 = time.time()
    spec = VISION.get(mode)
    if spec is None:
        return {'ok': False, 'error': f'unknown vision mode {mode}'}
    views = [v for v in (images or []) if isinstance(v, dict) and v.get('b64')]
    if not views:
        views = [{'label': '', 'b64': image_b64}] if image_b64 else []
    if not views or any(len(v['b64']) > MAX_IMAGE_B64 for v in views) or len(views) > 4:
        return {'ok': False, 'error': 'no camera frame' if not views else 'frames too large'}
    c = client()
    if c is None:
        return {'ok': False, 'error': f'no Anthropic API key: put it in {KEY_FILE}'}
    ctx = '\n'.join(f'{k}: {v}' for k, v in (context or {}).items() if v not in (None, ''))
    content = []
    for v in views:
        if v.get('label'):
            content.append({'type': 'text', 'text': str(v['label'])[:200]})
        mt = v.get('type') if v.get('type') in ('image/jpeg', 'image/png') else 'image/jpeg'
        content.append({'type': 'image', 'source': {'type': 'base64', 'media_type': mt, 'data': v['b64']}})
    content.append({'type': 'text', 'text': (f'Context:\n{ctx}\n\n' if ctx else '')
                    + ('Answer about these views.' if len(views) > 1 else 'Answer about this frame.')})
    try:
        resp = c.beta.messages.create(
            model=MODEL, max_tokens=spec.get('max_tokens', 2048),
            betas=['server-side-fallback-2026-07-01'], fallbacks='default',
            output_config={'effort': spec.get('effort', EFFORT), 'format': {'type': 'json_schema', 'schema': spec['schema']}},
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


# ------------------------------------------------------------------ speech-to-text (on the robot)
# Browsers' built-in speech recognition sends the audio to Google (Chrome) or fails outright ("network" in
# Brave and other non-Google Chromium builds, 2026-10-30). So the page records the audio and the robot
# transcribes it with Whisper (faster-whisper, int8, CPU), offline: works in any browser, audio never
# leaves the robot. Measured on the Pi with nav running, 2 threads: base.en ~360 MB peak, 11 s of speech
# in 8.4 s; tiny.en ~280 MB, 5.1 s. The model lives in a worker PROCESS started on the first command and
# ended after STT_IDLE s without one, so the memory goes back to zero when nobody uses voice.
STT_MODEL = os.environ.get('STT_MODEL', 'base.en')       # tiny.en: faster, a little worse on names
STT_THREADS = int(os.environ.get('STT_THREADS', '2'))    # leave cores for navigation
STT_IDLE = float(os.environ.get('STT_IDLE', '600'))
STT_DIR = os.path.expanduser('~/.cache/campus_bot/whisper')
MAX_AUDIO_B64 = 4 << 20
_stt_model = None                                         # inside the worker process only


def _stt_init(model, threads, root):
    global _stt_model
    from faster_whisper import WhisperModel
    _stt_model = WhisperModel(model, device='cpu', compute_type='int8', cpu_threads=threads, download_root=root)


def _stt_run(audio, suffix, prompt):
    """Decode with the system ffmpeg (16 kHz mono float32: faster-whisper's own PyAV path breaks on PyAV 19)
    from a temp file (Safari's mp4 needs a seekable input), then transcribe. Runs in the worker."""
    import subprocess
    import tempfile
    import numpy as np
    with tempfile.NamedTemporaryFile(suffix=suffix, dir='/dev/shm') as f:
        f.write(audio)
        f.flush()
        raw = subprocess.run(['ffmpeg', '-loglevel', 'error', '-i', f.name, '-f', 'f32le', '-ac', '1',
                              '-ar', '16000', 'pipe:1'], capture_output=True, timeout=30).stdout
    pcm = np.frombuffer(raw, np.float32)
    if len(pcm) < 1600:                                   # < 0.1 s: nothing recorded
        return ''
    segs, _info = _stt_model.transcribe(pcm, language='en', beam_size=1, vad_filter=True,
                                        initial_prompt=prompt)
    return ' '.join(s.text.strip() for s in segs).strip()


_stt = {'ex': None, 'last': 0.0, 'lock': threading.Lock()}


def _stt_reaper():
    while True:
        time.sleep(30)
        with _stt['lock']:
            if _stt['ex'] is not None and time.time() - _stt['last'] > STT_IDLE:
                _stt['ex'].shutdown(wait=False, cancel_futures=True)
                _stt['ex'] = None
                print(time.strftime('%H:%M:%S'), 'speech model unloaded (idle)', flush=True)


def transcribe(audio_b64, mime='', places=None):
    import base64
    import multiprocessing
    from concurrent.futures import ProcessPoolExecutor
    t0 = time.time()
    if not audio_b64 or len(audio_b64) > MAX_AUDIO_B64:
        return {'ok': False, 'error': 'no audio' if not audio_b64 else 'recording too long'}
    try:
        audio = base64.b64decode(audio_b64)
    except ValueError:
        return {'ok': False, 'error': 'bad audio'}
    suffix = '.mp4' if 'mp4' in (mime or '') else '.ogg' if 'ogg' in (mime or '') else '.webm'
    names = ', '.join(str(p) for p in (places or [])[:40])
    prompt = 'Commands for an office delivery robot. ' + (f'Places and people: {names}.' if names else '')
    with _stt['lock']:
        if _stt['ex'] is None:
            _stt['ex'] = ProcessPoolExecutor(1, mp_context=multiprocessing.get_context('spawn'),
                                             initializer=_stt_init, initargs=(STT_MODEL, STT_THREADS, STT_DIR))
            print(time.strftime('%H:%M:%S'), f'speech model {STT_MODEL} loading', flush=True)
        _stt['last'] = time.time()
        ex = _stt['ex']
    try:
        text = ex.submit(_stt_run, audio, suffix, prompt).result(timeout=90)
    except Exception as e:
        with _stt['lock']:                                # a broken worker: start fresh next time
            if _stt['ex'] is ex:
                ex.shutdown(wait=False, cancel_futures=True)
                _stt['ex'] = None
        return {'ok': False, 'error': f'speech recognition failed: {e.__class__.__name__}'}
    _stt['last'] = time.time()
    if not text:
        return {'ok': False, 'error': "didn't hear anything: hold the phone closer and speak after the beep"}
    return {'ok': True, 'text': text, 'ms': int((time.time() - t0) * 1000), 'model': STT_MODEL}


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
                                    'effort': EFFORT,
                                    'stt': {'model': STT_MODEL, 'loaded': _stt['ex'] is not None}})
        self._send(404, {'ok': False, 'error': 'not found'})

    def do_POST(self):
        if self.path not in ('/parse', '/vision', '/transcribe'):
            return self._send(404, {'ok': False, 'error': 'not found'})
        try:
            n = int(self.headers.get('Content-Length', 0) or 0)
            body = json.loads(self.rfile.read(min(n, max(MAX_IMAGE_B64, MAX_AUDIO_B64) + (1 << 16))) or b'{}')
            if not isinstance(body, dict):
                raise ValueError
        except ValueError:
            return self._send(400, {'ok': False, 'error': 'bad JSON body'})
        if self.path == '/transcribe':
            r = transcribe(body.get('audio_b64'), str(body.get('mime') or ''), body.get('places'))
            print(time.strftime('%H:%M:%S'), 'transcribe', 'ok' if r.get('ok') else 'FAIL',
                  json.dumps({'ms': r.get('ms'), 'text': r.get('text'), 'error': r.get('error')}), flush=True)
            return self._send(200, r)
        if self.path == '/vision':
            r = vision(str(body.get('mode', '')), body.get('image_b64'), body.get('context'), body.get('images'))
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
    threading.Thread(target=_stt_reaper, daemon=True).start()
    srv = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f'command service on http://{HOST}:{PORT}  (model {MODEL}, effort {EFFORT}; '
          f'API key {"found" if api_key() else "MISSING: " + KEY_FILE})', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    sys.exit(main())
