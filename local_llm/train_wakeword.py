"""
Train a custom "hey wolfwagen" wakeword model using OpenWakeWord features.

Reads WAV files from training_data/positive and training_data/negative,
extracts audio embeddings with OpenWakeWord's pre-trained feature extractor,
trains a small binary classifier, and exports to ONNX.

No heavy training dependencies required — uses only torch + openwakeword.

Usage:
    python3 train_wakeword.py
    python3 train_wakeword.py --data_dir ./training_data --steps 20000
"""

from __future__ import annotations

import argparse
import copy
import logging
import random
import wave
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch import optim
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler
from tqdm import tqdm

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

CLIP_SAMPLES  = 32000   # 2 s at 16 kHz
WINDOW_FRAMES = 16      # model input width (16 × 80 ms = 1.28 s)
VAL_SPLIT     = 0.15
BATCH_SIZE    = 256
MODEL_NAME    = "hey_wolfwagen"
OUTPUT_DIR    = Path(__file__).parent / "models"


# ---------------------------------------------------------------------------
# Model  (mirrors the DNN architecture inside openwakeword/train.py)
# ---------------------------------------------------------------------------

class _FCNBlock(nn.Module):
    def __init__(self, dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(dim, dim),
            nn.LayerNorm(dim),
            nn.ReLU(),
        )

    def forward(self, x):
        return self.net(x)


class WakewordModel(nn.Module):
    def __init__(self, input_frames: int = 16, input_dim: int = 96,
                 layer_dim: int = 128, n_blocks: int = 1):
        super().__init__()
        flat = input_frames * input_dim
        self.net = nn.Sequential(
            nn.Flatten(),
            nn.Linear(flat, layer_dim),
            nn.LayerNorm(layer_dim),
            nn.ReLU(),
            *[_FCNBlock(layer_dim) for _ in range(n_blocks)],
            nn.Linear(layer_dim, 1),
            nn.Sigmoid(),
        )

    def forward(self, x):
        return self.net(x)


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_wav(path: Path) -> np.ndarray:
    with wave.open(str(path), "r") as wf:
        raw = wf.readframes(wf.getnframes())
    return np.frombuffer(raw, dtype=np.int16)


def pad_or_trim(audio: np.ndarray, length: int) -> np.ndarray:
    if len(audio) >= length:
        return audio[:length]
    return np.pad(audio, (0, length - len(audio)))


def load_clips(directory: Path) -> np.ndarray:
    paths = sorted(directory.glob("*.wav"))
    if not paths:
        raise FileNotFoundError(f"No WAV files in {directory}")
    clips = np.stack([pad_or_trim(load_wav(p), CLIP_SAMPLES) for p in paths])
    logging.info(f"Loaded {len(clips)} clips from {directory}")
    return clips


def extract_features(clips: np.ndarray) -> np.ndarray:
    from openwakeword.utils import AudioFeatures
    F = AudioFeatures(inference_framework="onnx")
    features = F.embed_clips(clips, batch_size=64)
    logging.info(f"Features shape: {features.shape}")
    return features


# ---------------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------------

class WakewordDataset(Dataset):
    def __init__(self, features: np.ndarray, labels: np.ndarray, augment: bool = False):
        self.features = features.astype(np.float32)
        self.labels   = torch.tensor(labels, dtype=torch.float32)
        self.n_frames = features.shape[1]
        self.augment  = augment

    def __len__(self) -> int:
        return len(self.features)

    def __getitem__(self, idx):
        clip = self.features[idx]
        if self.n_frames >= WINDOW_FRAMES:
            start = random.randint(0, self.n_frames - WINDOW_FRAMES)
            window = clip[start : start + WINDOW_FRAMES].copy()
        else:
            pad = np.zeros((WINDOW_FRAMES - self.n_frames, 96), dtype=np.float32)
            window = np.vstack([clip, pad])
        if self.augment:
            # Small feature-space noise — breaks embedding uniformity from synthetic TTS
            window += np.random.normal(0, 0.01, window.shape).astype(np.float32)
        return torch.tensor(window), self.labels[idx]


def make_loaders(pos_feat, neg_feat):
    def split(arr):
        n_val = max(1, int(len(arr) * VAL_SPLIT))
        idx = np.random.permutation(len(arr))
        return arr[idx[n_val:]], arr[idx[:n_val]]

    pos_tr, pos_val = split(pos_feat)
    neg_tr, neg_val = split(neg_feat)

    tr_feat = np.vstack([pos_tr, neg_tr])
    tr_lbl  = np.concatenate([np.ones(len(pos_tr)), np.zeros(len(neg_tr))])

    val_feat = np.vstack([pos_val, neg_val])
    val_lbl  = np.concatenate([np.ones(len(pos_val)), np.zeros(len(neg_val))])

    # Balance training batches via weighted sampling (fixes ~1:4 pos/neg imbalance)
    class_counts = [int((tr_lbl == 0).sum()), int((tr_lbl == 1).sum())]
    sample_weights = [1.0 / class_counts[int(l)] for l in tr_lbl]
    sampler = WeightedRandomSampler(sample_weights, len(tr_lbl), replacement=True)

    train_dl = DataLoader(WakewordDataset(tr_feat, tr_lbl, augment=True),
                          batch_size=BATCH_SIZE, sampler=sampler, drop_last=True)
    val_dl   = DataLoader(WakewordDataset(val_feat, val_lbl, augment=False),
                          batch_size=BATCH_SIZE, shuffle=False)

    logging.info(f"Train: {len(tr_feat)} samples  |  Val: {len(val_feat)} samples")
    return train_dl, val_dl


# ---------------------------------------------------------------------------
# Training loop
# ---------------------------------------------------------------------------

def cosine_lr(step: int, warmup: int, total: int, lr_max: float = 1e-3) -> float:
    if step < warmup:
        return lr_max * step / max(1, warmup)
    progress = (step - warmup) / max(1, total - warmup)
    return lr_max * 0.5 * (1.0 + np.cos(np.pi * progress))


def train(data_dir: Path, steps: int) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logging.info(f"Device: {device}")

    # --- features ---
    pos_clips = load_clips(data_dir / "positive")
    neg_clips = load_clips(data_dir / "negative")

    logging.info("Extracting features from positive clips...")
    pos_feat = extract_features(pos_clips)

    logging.info("Extracting features from negative clips...")
    neg_feat = extract_features(neg_clips)

    train_dl, val_dl = make_loaders(pos_feat, neg_feat)

    # --- model ---
    model = WakewordModel().to(device)
    optimizer = optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.BCELoss()
    warmup = steps // 5

    best_val_loss = float("inf")
    best_state    = copy.deepcopy(model.state_dict())

    train_iter = iter(train_dl)
    model.train()

    for step in tqdm(range(steps), desc="Training"):
        # cycle through the dataloader
        try:
            x, y = next(train_iter)
        except StopIteration:
            train_iter = iter(train_dl)
            x, y = next(train_iter)

        x, y = x.to(device), y.to(device)

        # lr schedule
        lr = cosine_lr(step, warmup, steps)
        for g in optimizer.param_groups:
            g["lr"] = lr

        optimizer.zero_grad()
        pred = model(x).squeeze(1)
        loss = loss_fn(pred, y)
        loss.backward()
        optimizer.step()

        # validation every 10 % of steps
        if step > 0 and step % max(1, steps // 10) == 0:
            model.eval()
            val_losses, val_correct, val_total = [], 0, 0
            with torch.no_grad():
                for xv, yv in val_dl:
                    xv, yv = xv.to(device), yv.to(device)
                    pv = model(xv).squeeze(1)
                    val_losses.append(loss_fn(pv, yv).item())
                    val_correct += ((pv >= 0.5).float() == yv).sum().item()
                    val_total   += len(yv)
            vl  = np.mean(val_losses)
            acc = val_correct / val_total
            logging.info(f"step {step:>6d}  val_loss={vl:.4f}  val_acc={acc:.3f}  lr={lr:.2e}")
            if vl < best_val_loss:
                best_val_loss = vl
                best_state = copy.deepcopy(model.state_dict())
            model.train()

    # restore best checkpoint
    model.load_state_dict(best_state)
    model.eval().to("cpu")

    # --- export to ONNX ---
    onnx_path = OUTPUT_DIR / f"{MODEL_NAME}.onnx"
    dummy = torch.rand(1, WINDOW_FRAMES, 96)
    torch.onnx.export(
        model, (dummy,), str(onnx_path),
        input_names=["input"],
        output_names=[MODEL_NAME],
        opset_version=13,
    )
    logging.info(f"Saved ONNX model → {onnx_path}")
    logging.info("Run voice_assistant.py — it will auto-load the new wakeword model.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", type=Path,
                        default=Path(__file__).parent / "training_data")
    parser.add_argument("--steps", type=int, default=20000,
                        help="Training steps (default 20000; use 50000 for higher quality)")
    args = parser.parse_args()
    train(args.data_dir, args.steps)
