#!/usr/bin/env python3
# RECONSTRUCTED from RECOVERY.md/CLAUDE.md (2026-10-01) — not yet run on the robot.
"""Face enrolment service: a colleague opens the page on their phone, opts in, and the phone captures
~15 frames over 5 head angles. The Pi embeds them and keeps only the embeddings (encrypted, via
face_store.py). Images are decoded in memory and discarded; nothing is written to disk or logged.

    ENROLL_TOKEN=<shared code> ~/.venvs/face/bin/python face/enroll_service.py   # 0.0.0.0:8000
    ngrok http 8000          # phones only allow the camera over HTTPS; open the https URL it prints

    GET    /                    enrolment page (inline HTML/JS, no external assets)
    POST   /enroll              person_id, consent, files[] (JPEG)  -> stores embeddings
    GET    /people              names + count only
    DELETE /people/{person_id}  remove someone (the opt-out)
Writes need header `X-Enroll-Token: <ENROLL_TOKEN>` when ENROLL_TOKEN is set. Set it whenever ngrok is
up: the URL is public. The ngrok authtoken lives in ngrok's own config; never put it in this repo.
"""
import hmac
import os
import sys
import threading
from typing import List

import cv2
import numpy as np
from fastapi import FastAPI, File, Form, Header, HTTPException, UploadFile
from fastapi.responses import HTMLResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from engine import Engine  # noqa: E402
from face_store import FaceStore, check_person_id  # noqa: E402

MIN_GOOD = 5            # reject the enrolment below this many usable faces
MAX_FILES = 30
MAX_BYTES = 4 * 1024 * 1024
MIN_DET_SCORE = 0.6
MIN_CONSISTENCY = 0.35  # frames this far from the others are probably someone else in shot
DET_SIZE = int(os.environ.get('FACE_DET_SIZE', '640'))

app = FastAPI(title='Campus Bot face enrolment', docs_url=None, redoc_url=None, openapi_url=None)
store = FaceStore()
_engine = None
_engine_lock = threading.Lock()


def engine():
    global _engine
    with _engine_lock:
        if _engine is None:
            _engine = Engine(det_size=DET_SIZE)
        return _engine


def check_token(token):
    want = os.environ.get('ENROLL_TOKEN', '')
    if want and not hmac.compare_digest((token or '').encode(), want.encode()):
        raise HTTPException(status_code=401, detail='wrong or missing access code')


def pid_or_400(person_id):
    try:
        return check_person_id(person_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get('/', response_class=HTMLResponse)
def page():
    return HTMLResponse(PAGE, headers={'Cache-Control': 'no-store'})


@app.get('/people')
def people():
    names = [pid for pid, _n, _at in store.list_people()]
    return {'count': len(names), 'people': names}


@app.delete('/people/{person_id}')
def delete_person(person_id: str, x_enroll_token: str = Header(default='')):
    check_token(x_enroll_token)
    pid = pid_or_400(person_id)
    if not store.delete(pid):
        raise HTTPException(status_code=404, detail=f'{pid} is not enrolled')
    return {'deleted': pid}


# Plain `def`: FastAPI runs it in a worker thread, so CPU-bound embedding doesn't block the event loop.
@app.post('/enroll')
def enroll(person_id: str = Form(...), consent: bool = Form(False), files: List[UploadFile] = File(...),
           x_enroll_token: str = Header(default='')):
    check_token(x_enroll_token)
    pid = pid_or_400(person_id)
    if not consent:
        raise HTTPException(status_code=400, detail='consent checkbox not ticked: nothing stored')
    if len(files) > MAX_FILES:
        raise HTTPException(status_code=400, detail=f'too many frames (max {MAX_FILES})')

    eng = engine()
    embs, rejected = [], {'unreadable': 0, 'no_face': 0, 'low_quality': 0, 'inconsistent': 0}
    for uf in files:
        buf = uf.file.read(MAX_BYTES + 1)
        img = None
        if len(buf) <= MAX_BYTES:
            img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
        del buf
        if img is None:
            rejected['unreadable'] += 1
            continue
        with _engine_lock:   # one inference at a time on the Pi
            emb, _bbox, det, _ms = eng.embed(img)
        del img              # images never outlive this loop
        if emb is None:
            rejected['no_face'] += 1
        elif det < MIN_DET_SCORE:
            rejected['low_quality'] += 1
        else:
            embs.append(emb)

    # Drop frames that don't agree with the rest (someone walked into shot).
    if len(embs) >= 3:
        e = np.stack(embs)
        sims = e @ e.T
        agree = (sims.sum(axis=1) - 1.0) / (len(e) - 1)
        keep = agree >= MIN_CONSISTENCY
        rejected['inconsistent'] = int((~keep).sum())
        embs = list(e[keep])

    if len(embs) < MIN_GOOD:
        raise HTTPException(status_code=422, detail={
            'message': f'only {len(embs)} usable faces (need {MIN_GOOD}); try better light, face the camera',
            'good': len(embs), 'received': len(files), 'rejected': rejected})
    n = store.enroll(pid, embs, consent=True)
    return {'person_id': pid, 'stored_embeddings': n, 'received': len(files), 'rejected': rejected}


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Campus Bot enrolment</title>
<style>
:root{--bg:#fff;--fg:#111;--mut:#666;--acc:#0a66c2;--bad:#b00020;--ok:#137333;--card:#f3f4f6}
@media (prefers-color-scheme:dark){:root{--bg:#111;--fg:#eee;--mut:#aaa;--acc:#5aa9ff;--bad:#ff6b81;--ok:#5bd17a;--card:#1d1f23}}
*{box-sizing:border-box}body{margin:0;padding:16px;font:16px/1.4 system-ui,sans-serif;background:var(--bg);color:var(--fg);max-width:520px;margin:auto}
h1{font-size:1.3em}label{display:block;margin:10px 0 4px}input[type=text],input[type=password]{width:100%;padding:10px;font-size:16px;border:1px solid var(--mut);border-radius:8px;background:var(--bg);color:var(--fg)}
.card{background:var(--card);padding:12px;border-radius:10px;margin:12px 0}
button{width:100%;padding:14px;font-size:17px;border:0;border-radius:10px;background:var(--acc);color:#fff;margin-top:10px}
button:disabled{opacity:.45}button.sec{background:transparent;color:var(--bad);border:1px solid var(--bad)}
video{width:100%;border-radius:10px;background:#000;transform:scaleX(-1)}
#step{font-size:1.4em;font-weight:600;text-align:center;margin:8px 0}#msg{white-space:pre-wrap}
.ok{color:var(--ok)}.bad{color:var(--bad)}small{color:var(--mut)}
</style></head><body>
<h1>Campus Bot face enrolment</h1>
<div class="card"><small>The delivery robot checks your face when it hands over a parcel. This page takes
about 15 photos of your face; the robot keeps only an encrypted numeric template (no photos are stored),
used only to confirm deliveries to you. You can delete it at any time below.</small></div>
<label for="pid">Your ID (lowercase, e.g. hemant)</label><input id="pid" type="text" autocapitalize="none" autocomplete="off">
<label for="tok">Access code (if you were given one)</label><input id="tok" type="password" autocomplete="off">
<label><input id="consent" type="checkbox"> I opt in to face verification for deliveries and agree to the
template being stored on the robot until I delete it.</label>
<button id="start">Start camera</button>
<video id="v" playsinline muted hidden></video>
<div id="step"></div>
<button id="go" hidden>Capture</button>
<p id="msg"></p>
<div class="card"><b>Delete my data</b><br><small>Uses the ID and access code above.</small>
<button id="del" class="sec">Delete my face template</button></div>
<canvas id="c" hidden></canvas>
<script>
const STEPS=[["Look straight at the camera","front"],["Turn slightly to your LEFT","left"],
 ["Turn slightly to your RIGHT","right"],["Tilt your chin slightly UP","up"],["Tilt your chin slightly DOWN","down"]];
const PER_STEP=3, $=id=>document.getElementById(id);
let stream=null;
const msg=(t,c)=>{$('msg').textContent=t;$('msg').className=c||''};
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
const pid=()=>$('pid').value.trim().toLowerCase();
const hdr=()=>{const t=$('tok').value;return t?{'X-Enroll-Token':t}:{}};
function valid(){if(!/^[a-z0-9][a-z0-9._-]{0,63}$/.test(pid())){msg('Enter an ID: letters/numbers only.','bad');return false}return true}
$('start').onclick=async()=>{
 if(!valid())return;
 if(!$('consent').checked){msg('Tick the opt-in box first. Nothing is captured without it.','bad');return}
 if(!navigator.mediaDevices||!window.isSecureContext){msg('Camera needs HTTPS. Open the https:// ngrok link.','bad');return}
 try{stream=await navigator.mediaDevices.getUserMedia({video:{facingMode:'user',width:{ideal:1280},height:{ideal:720}},audio:false})}
 catch(e){msg('Camera blocked: '+e.message,'bad');return}
 $('v').srcObject=stream;$('v').hidden=false;await $('v').play();
 $('start').hidden=true;$('go').hidden=false;$('step').textContent='Good light on your face, then tap Capture';msg('');
};
function grab(){const v=$('v'),c=$('c'),s=Math.min(1,960/Math.max(v.videoWidth,v.videoHeight));
 c.width=Math.round(v.videoWidth*s);c.height=Math.round(v.videoHeight*s);
 c.getContext('2d').drawImage(v,0,0,c.width,c.height);
 return new Promise(r=>c.toBlob(r,'image/jpeg',0.9))}
$('go').onclick=async()=>{
 if(!valid()||!$('consent').checked){msg('ID and opt-in box are required.','bad');return}
 $('go').disabled=true;const fd=new FormData();fd.append('person_id',pid());fd.append('consent','true');
 for(let i=0;i<STEPS.length;i++){
  $('step').textContent=`${i+1}/${STEPS.length}: ${STEPS[i][0]}`;await sleep(1800);
  for(let k=0;k<PER_STEP;k++){fd.append('files',await grab(),`${STEPS[i][1]}_${k}.jpg`);await sleep(250)}
 }
 stream.getTracks().forEach(t=>t.stop());$('v').hidden=true;$('step').textContent='Uploading…';
 try{const r=await fetch('enroll',{method:'POST',body:fd,headers:hdr()});const j=await r.json();
  if(r.ok){$('step').textContent='Done';msg(`Enrolled ${j.person_id}: ${j.stored_embeddings} templates stored. Photos discarded.`,'ok')}
  else{$('step').textContent='Not enrolled';msg((j.detail&&j.detail.message)||JSON.stringify(j.detail),'bad')}}
 catch(e){msg('Upload failed: '+e.message,'bad')}
 $('go').hidden=true;$('go').disabled=false;$('start').hidden=false;$('start').textContent='Try again';
};
$('del').onclick=async()=>{
 if(!valid())return;if(!confirm(`Delete the face template for "${pid()}"?`))return;
 const r=await fetch('people/'+encodeURIComponent(pid()),{method:'DELETE',headers:hdr()});const j=await r.json();
 msg(r.ok?`Deleted ${j.deleted}.`:(j.detail||'failed'),r.ok?'ok':'bad');
};
</script></body></html>
"""


def main():
    import uvicorn
    if not os.environ.get('ENROLL_TOKEN'):
        print('WARNING: ENROLL_TOKEN not set: anyone with the URL can enrol or delete people. '
              'Set it before running ngrok.', file=sys.stderr)
    engine()   # load the model before the first phone connects
    uvicorn.run(app, host='0.0.0.0', port=int(os.environ.get('ENROLL_PORT', '8000')))


if __name__ == '__main__':
    main()
