# RECONSTRUCTED from RECOVERY.md/CLAUDE.md (2026-10-01) — not yet run on the robot.
"""Thin InsightFace wrapper: buffalo_s (SCRFD detector + ArcFace), CPU only.

    from engine import Engine, cosine
    eng = Engine(det_size=640)                # 320 is faster; the last p95 was ~900 ms at 640
    emb, bbox, det_score, ms = eng.embed(frame_bgr)   # largest face; emb is L2-normalised or None

The model downloads itself to ~/.insightface/models/buffalo_s on first use (~120 MB).
Only the detection and recognition models are loaded (no age/gender/landmarks: faster).
"""
import os
import time
import warnings

import numpy as np

MODEL = 'buffalo_s'
DET_SIZES = (320, 640)
MIN_FACE_PX = 40   # smaller faces embed badly; treat as "no face"

warnings.filterwarnings('ignore', category=FutureWarning, module='insightface')


class Engine:
    def __init__(self, det_size=640, det_thresh=0.5):
        if det_size not in DET_SIZES:
            raise ValueError(f'det_size must be one of {DET_SIZES}')
        from insightface.app import FaceAnalysis   # slow import; keep `--help` fast
        self.det_size = det_size
        self.app = FaceAnalysis(name=MODEL, root=os.path.expanduser('~/.insightface'),
                                allowed_modules=['detection', 'recognition'],
                                providers=['CPUExecutionProvider'])
        self.app.prepare(ctx_id=-1, det_thresh=det_thresh, det_size=(det_size, det_size))

    def embed(self, frame_bgr):
        """(embedding (512,) float32 L2-normalised or None, bbox [x1,y1,x2,y2] or None, det_score, ms)."""
        t0 = time.perf_counter()
        if frame_bgr is None or getattr(frame_bgr, 'ndim', 0) != 3:
            return None, None, 0.0, 0.0
        faces = self.app.get(frame_bgr)
        ms = (time.perf_counter() - t0) * 1000.0
        if not faces:
            return None, None, 0.0, ms
        f = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
        bbox = [float(x) for x in f.bbox]
        if min(bbox[2] - bbox[0], bbox[3] - bbox[1]) < MIN_FACE_PX or f.embedding is None:
            return None, bbox, float(f.det_score), ms
        return f.normed_embedding.astype(np.float32), bbox, float(f.det_score), ms


    def analyze(self, frame_bgr, embed=True):
        """Largest face with its 5 landmarks (for head pose), for the guided enrolment.
        embed=False runs the detector only (fast, for the live preview).
        Returns dict(n_faces, bbox, det, kps (5,2) or None, emb or None, ms)."""
        t0 = time.perf_counter()
        out = {'n_faces': 0, 'bbox': None, 'det': 0.0, 'kps': None, 'emb': None, 'ms': 0.0}
        if frame_bgr is None or getattr(frame_bgr, 'ndim', 0) != 3:
            return out
        if embed:
            faces = self.app.get(frame_bgr)
            if faces:
                f = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
                small = min(f.bbox[2] - f.bbox[0], f.bbox[3] - f.bbox[1]) < MIN_FACE_PX
                out.update(n_faces=len(faces), bbox=[float(x) for x in f.bbox], det=float(f.det_score),
                           kps=np.asarray(f.kps, dtype=np.float32),
                           emb=None if small or f.embedding is None else f.normed_embedding.astype(np.float32))
        else:
            bboxes, kpss = self.app.det_model.detect(frame_bgr, max_num=0, metric='default')
            if bboxes is not None and len(bboxes):
                i = int(np.argmax((bboxes[:, 2] - bboxes[:, 0]) * (bboxes[:, 3] - bboxes[:, 1])))
                out.update(n_faces=len(bboxes), bbox=[float(x) for x in bboxes[i, :4]],
                           det=float(bboxes[i, 4]), kps=None if kpss is None else kpss[i].astype(np.float32))
        out['ms'] = (time.perf_counter() - t0) * 1000.0
        return out


def cosine(a, b):
    """Cosine similarity. b may be (512,) or (n, 512); returns a float or an (n,) array."""
    a = np.asarray(a, dtype=np.float32)
    b = np.asarray(b, dtype=np.float32)
    a = a / max(np.linalg.norm(a), 1e-9)
    if b.ndim == 1:
        return float(np.dot(a, b / max(np.linalg.norm(b), 1e-9)))
    return (b @ a) / np.maximum(np.linalg.norm(b, axis=1), 1e-9)


def best_score(emb, enrolled):
    """Max cosine over a person's enrolled embeddings (n, 512)."""
    return float(np.max(cosine(emb, enrolled)))
