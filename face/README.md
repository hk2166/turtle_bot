<!-- RECONSTRUCTED from RECOVERY.md/CLAUDE.md (2026-10-01) — not yet run on the robot. -->
# Face verification

Confirms the recipient at handover: 1:1 check of the person at the crate against the claimed
recipient's enrolment (OTP is the fallback). InsightFace **buffalo_s** (SCRFD + ArcFace) on the Pi's
CPU, best-of-N frames from the mast Brio. Rebuilt from the spec on 2026-10-01; **not yet run on the
robot or with a real camera**. The 3 colleagues enrolled before the loss must re-enrol unless the old
card gives back both `faces.enc` and `faces.key`.

| file | what |
|---|---|
| `face_store.py` | Fernet-encrypted embedding store, `~/.campus_bot/faces.enc` + `faces.key` |
| `engine.py` | InsightFace wrapper: `embed(frame) -> (embedding, bbox, det_score, ms)` |
| `verify.py` | best-of-N 1:1 verification, CLI and `verify(person_id, frames)` |
| `enroll_service.py` | FastAPI phone enrolment page + `/enroll`, `/people`, `DELETE /people/{id}` |
| `calibrate.py` | collect scored frames, then TAR/FAR sweep, timing, desk-vs-mast, best-of-N |

## Setup
```bash
python3 -m venv ~/.venvs/face && ~/.venvs/face/bin/pip install -r face/requirements.txt   # once
source ~/.venvs/face/bin/activate
```
The model (~120 MB) downloads to `~/.insightface/models/buffalo_s` on first use. Stop
`run/camera.sh` (or anything else holding `/dev/video0`) before using the camera here.

## Enrol (phone)
```bash
ENROLL_TOKEN=<pick a code> python face/enroll_service.py     # 0.0.0.0:8000
ngrok http 8000                                               # second terminal
```
Phones only give the page the camera over HTTPS, so send people the `https://….ngrok…` URL plus
the code. The page asks for their ID (lowercase, e.g. `lakshaysharma`), needs the opt-in box ticked,
then walks them through front / slight left / right / up / down (3 frames each). The server needs ≥ 5
usable faces and drops frames that don't match the rest. Re-enrolling replaces the old template.
The ngrok authtoken stays in ngrok's own config (`ngrok config add-authtoken`); never put it in this
repo, a script or a log. Stop ngrok when enrolment is done. Without `ENROLL_TOKEN`, anyone with the URL can
enrol or delete people. `GET /people` (names only) is open either way.

`python face/face_store.py` lists who is enrolled. `--delete <id>` removes someone.

## Verify
```bash
python face/verify.py hemant                         # 5 frames from /dev/video0, threshold 0.42
python face/verify.py hemant --det-size 320 --frames 5
python face/verify.py hemant --images a.jpg b.jpg    # from files
```
Exit code 0 = accept, 1 = reject, 2 = error. Stops at the first frame ≥ threshold; the 2 s budget is checked
between frames. **0.42 is a placeholder** until calibration has impostor data.

## Self-service enrolment for the office (added 2026-10-22) — the way to enrol people now

Anyone on the office WiFi opens **`https://resp-desktop.local:8443/enroll`** (or taps ☺ REGISTER MY
FACE on the call page; `http://resp-desktop.local:8088/enroll` redirects there) on their **own phone or
laptop**: ID, consent box, Start, then they roll their head slowly in a circle while a Face-ID-style ring
of 12 head directions fills in (~30 s). The browser only captures and displays; it posts ~4 small frames
a second, the robot embeds each in memory, updates the ring and sends the next hint. Only the final
embeddings are stored (encrypted). Logic: `enroll_session.py`; page: `enroll_page.html`; served by
`face_service.py` (starts at boot via `run/autostart.sh`).

- **HTTPS is required** (browsers only give a page the camera over https). The service makes a
  self-signed cert for `<host>.local` + its LAN IPs in `~/.campus_bot/tls/` (never commit), so each
  person taps through one "not private → proceed / visit this website" warning the first time.
- **Firewall:** ufw is on — open the port once: `sudo ufw allow 8443/tcp`.
- **Safety:** consent required; re-enrolling an existing ID only works if the new face matches the
  old one (nobody can take over someone's ID); "Delete my face data" deletes only after a live face
  check against that ID; up to 4 people at once; optional shared code with `ENROLL_CODE=…`.
- At the robot instead (no phone): `face/enroll_local.py <id> --consent`.
- The face camera is found automatically (the first USB camera that isn't the Logitech Brio);
  `FACE_CAM=/dev/videoN` or `setup/10_face_cam.sh` (`/dev/face_cam`) override it.
- **Orientation:** its mount holds it upside-down, so frames are turned by `rotate:` in
  `config/face_cam.yaml` (180) before anything sees them. The detector can't find a face that is
  sideways or upside-down, so after remounting, re-measure and update it.
- **Dashboard:** EYES → ☺ FACE / BOTH shows this camera live with a box per face, green with the
  name when recognised, amber "not recognised" otherwise (admin only; `/face_frame.jpg`).

## Live service — identify + verify during delivery (added 2026-10-22)

`verify.py`/`identify.py` load the model every run (seconds), which blows the 2 s handover budget. For
the robot, run the **warm service** instead: it loads buffalo_s once, owns the **second** camera, and
answers over localhost HTTP. The delivery flow calls it through `nodes/face_client.py` (pure stdlib,
degrades to OTP/manual if the service or camera is down).

```
run/face.sh on                       # starts face/face_service.py in the face venv → 127.0.0.1:8070
curl -s localhost:8070/health        # model_loaded, camera.available, enrolled count
curl -s localhost:8070/people        # enrolled ids
python3 nodes/face_client.py identify # 1:N: who is at the camera?   (from the ROS/dashboard python)
python3 nodes/face_client.py verify hemant   # 1:1: is it hemant?
```

Endpoints: `GET /health`, `GET /people`, `POST /verify {person_id}` (1:1 — the handover check),
`POST /identify {}` (1:N — who loaded / stronger handover). Both also take `n`, `threshold`,
`det_size` (identify: `margin`, `top`), and `{"images":[...]}` to run against files for testing.

- **`identify.py`** is the 1:N counterpart to `verify.py`: best-of-N, same 2 s budget, but a match must
  also beat the runner-up by `MARGIN` (0.04) so two look-alikes never silently swap.
- **Second camera:** `setup/10_face_cam.sh` (run once, with sudo, after plugging the HIKVISION in) pins
  it to `/dev/face_cam` so it never grabs the Brio's `/dev/video0`. `nodes/camera_node.py` keeps the
  Brio for AprilTags + preview; the face service uses only `/dev/face_cam`. No device contention.
- **det_size:** 640 is accurate for a normally-framed face (~0.5–1 m from the 1080p cam) but misses a
  face that fills a tiny crop; 320 is faster and catches small crops. Default 640 (`FACE_DET_SIZE`).

Proven end-to-end in a throwaway store (enroll → identify → verify: right person 1.00, strangers ~0.10).
**Not yet run on the robot with the real second camera** (mount being printed, 2026-10-22). The
threshold is still the 0.42 placeholder until `calibrate.py` has impostor data.

## Calibrate (what's still open, see RECOVERY.md)
```bash
python face/calibrate.py collect --person hemant --label genuine  --session mast --frames 40
python face/calibrate.py collect --person hemant --label impostor --session mast --frames 40   # someone else
python face/calibrate.py collect --person hemant --label genuine  --session mast --det-size 320
python face/calibrate.py analyze
python face/calibrate.py analyze --relabel lakshaysharma=impostor     # if Lakshay's frames were someone else
```
1. Collect **impostor** rows: the sweep can't recommend a threshold without them (the last run had
   120 genuine, 0 impostor). Use several people claiming each enrolled person.
2. Settle the `lakshaysharma` ~0.18 question with Hemant: relabel (`--relabel`) or re-enrol him.
3. Compare `--det-size 320` with 640: the last p95 was ~900 ms and the worst frame took over 2 s.
4. Put the recommended threshold into `verify.py`'s `THRESHOLD`.

Rows go to `~/.campus_bot/calib/scores.csv`. It holds scores only, no images.

## Privacy rules
- No images are kept. Uploaded frames are decoded in memory, embedded and dropped. Only embeddings
  are stored, encrypted.
- Enrolment is opt-in (explicit checkbox, checked again server-side). Anyone can remove themselves
  from the page, or via `DELETE /people/{id}` or `face_store.py --delete`.
- **Never commit `~/.campus_bot/`** (`faces.enc`, `faces.key`, calibration CSVs) or any ngrok token.
  Losing `faces.key` means everyone re-enrols. Back it up somewhere other than git.
