#!/usr/bin/env python3
"""Trains AcousticUUVController on (sonar rolling-window, ground-truth
position) data logged by training_data_logger.py (corrected version --
logs gt_x/gt_y/gt_z, not thruster commands, matching what the model
actually outputs; see that file's docstring for why).

This is a small, honest checkpoint: a handful of corridor-walk runs is a
toy amount of data for a CNN+TCN, not a claim of a production-ready
controller (matches the project's own stated scope in
history_perception.md -- "proves the wiring", now "proves the wiring AND
that training measurably reduces baseline ATE", still not a trained
production model). Supervises the model to predict the vehicle's true
position (relative to spawn) from the trailing time_steps sonar frames,
matching how sic_slam_graph_backend.py actually uses its output (a
position correction blended into its own spawn-relative dead-reckoned
frame -- see training_data_logger.py's docstring).
"""
import argparse
import csv
import sys

import numpy as np
import torch
import torch.nn as nn

from sic_slam.model import AcousticUUVController


def load_run(csv_path, num_samples):
    with open(csv_path) as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = list(reader)
    sonar_cols = [i for i, h in enumerate(header) if h.startswith('sonar_')]
    gt_cols = [header.index('gt_x'), header.index('gt_y'), header.index('gt_z')]
    sonar = np.array([[float(row[i]) for i in sonar_cols] for row in rows], dtype=np.float32) / 255.0
    gt = np.array([[float(row[i]) for i in gt_cols] for row in rows], dtype=np.float32)
    if sonar.shape[1] != num_samples:
        raise ValueError(
            f"{csv_path}: sonar row width {sonar.shape[1]} != expected num_samples={num_samples} "
            "(must match beam_count the run was launched with)")
    return sonar, gt


def build_windows(sonar, gt, time_steps):
    """Each window is time_steps consecutive sonar frames; label is the
    ground-truth position at the window's LAST frame (i.e. "where am I
    now, given what I've seen recently" -- matches the graph backend
    treating the prediction as a current-position correction)."""
    n = len(sonar)
    if n < time_steps:
        return np.zeros((0, time_steps, sonar.shape[1]), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)
    X = np.stack([sonar[i:i + time_steps] for i in range(n - time_steps + 1)])
    y = gt[time_steps - 1:]
    return X.astype(np.float32), y.astype(np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_paths', nargs='+')
    ap.add_argument('--time_steps', type=int, default=10)
    ap.add_argument('--num_samples', type=int, default=64)
    ap.add_argument('--epochs', type=int, default=100)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--val_frac', type=float, default=0.2)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--output', default='uuv_controller.pth')
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)

    all_X, all_y = [], []
    for path in args.csv_paths:
        sonar, gt = load_run(path, args.num_samples)
        X, y = build_windows(sonar, gt, args.time_steps)
        print(f'{path}: {len(sonar)} frames -> {len(X)} windows')
        all_X.append(X)
        all_y.append(y)
    X = np.concatenate(all_X, axis=0)
    y = np.concatenate(all_y, axis=0)
    if len(X) < 20:
        print(f'FAIL: only {len(X)} total training windows -- collect more/longer runs first.')
        sys.exit(1)

    idx = rng.permutation(len(X))
    n_val = max(1, int(len(X) * args.val_frac))
    val_idx, train_idx = idx[:n_val], idx[n_val:]

    X_train = torch.from_numpy(X[train_idx])
    y_train = torch.from_numpy(y[train_idx])
    X_val = torch.from_numpy(X[val_idx])
    y_val = torch.from_numpy(y[val_idx])

    model = AcousticUUVController(time_steps=args.time_steps, num_samples=args.num_samples)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.MSELoss()

    print(f'Training on {len(X_train)} windows, validating on {len(X_val)} ({len(X)} total)')
    for epoch in range(args.epochs):
        model.train()
        opt.zero_grad()
        pred = model(X_train)
        loss = loss_fn(pred, y_train)
        loss.backward()
        opt.step()

        if epoch % 10 == 0 or epoch == args.epochs - 1:
            model.eval()
            with torch.no_grad():
                val_pred = model(X_val)
                val_rmse = torch.sqrt(torch.mean((val_pred - y_val) ** 2)).item()
            print(f'epoch {epoch:3d}  train_mse={loss.item():.4f}  val_rmse={val_rmse:.4f} m')

    torch.save(model.state_dict(), args.output)
    print(f'Saved checkpoint: {args.output}')


if __name__ == '__main__':
    main()
