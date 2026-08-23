import torch
import torch.nn as nn


class AcousticUUVController(nn.Module):
    """1D-CNN spatial extractor + dilated causal TCN + control head.

    Maps a rolling (time_steps, num_samples) sonar intensity buffer to a
    3D landmark/displacement vector. Architecture per novelty.md; unchanged
    from the TensorRT version other than dropping the thruster head sizing
    (num_thrusters=3 here, since the perception bridge only wants an
    [x, y, z] landmark point, not raw thruster commands).
    """

    def __init__(self, time_steps=50, num_samples=500, output_dim=3):
        super().__init__()
        self.time_steps = time_steps
        self.num_samples = num_samples

        self.spatial_cnn = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=7, stride=2, padding=3),
            nn.BatchNorm1d(16), nn.ReLU(), nn.MaxPool1d(2),
            nn.Conv1d(16, 32, kernel_size=5, stride=2, padding=2),
            nn.BatchNorm1d(32), nn.ReLU(), nn.MaxPool1d(2),
            nn.Conv1d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm1d(64), nn.ReLU(),
            nn.AdaptiveAvgPool1d(1),
        )

        self.temporal_tcn = nn.Sequential(
            nn.Conv1d(64, 64, kernel_size=3, padding=1, dilation=1),
            nn.BatchNorm1d(64), nn.ReLU(),
            nn.Conv1d(64, 128, kernel_size=3, padding=2, dilation=2),
            nn.BatchNorm1d(128), nn.ReLU(),
            nn.Conv1d(128, 128, kernel_size=3, padding=4, dilation=4),
            nn.BatchNorm1d(128), nn.ReLU(),
            nn.AdaptiveAvgPool1d(1),
        )

        self.head = nn.Sequential(
            nn.Linear(128, 64), nn.ReLU(), nn.Dropout(0.2),
            nn.Linear(64, output_dim),
        )

    def forward(self, x):
        batch, time_steps, samples = x.shape
        x = x.view(batch * time_steps, 1, samples)
        x = self.spatial_cnn(x)
        x = x.view(batch, time_steps, 64).transpose(1, 2)
        x = self.temporal_tcn(x).view(batch, -1)
        return self.head(x)


if __name__ == "__main__":
    # ponytail: minimal shape self-check, no trained-weight assertions.
    model = AcousticUUVController(time_steps=50, num_samples=500)
    out = model(torch.randn(2, 50, 500))
    assert out.shape == (2, 3), out.shape
    print("AcousticUUVController forward OK:", out.shape)
