"""Run the published DRL-VO policy from its own checkpoint.

The checkpoint is a stable-baselines3 PPO model saved in 2021. Loading it with
the modern stable-baselines3 (2.x) rebuilds the right architecture but renames
the feature extractor, and running it there would silently use the wrong
activation. Both are avoided by rebuilding the two heads by hand from the raw
state dict: the network is small and its shape is fixed, so a handful of lines
is steadier than a version-dependent loader.

Two figures of the original are reproduced deliberately:

- the MLP activation is ``Tanh``. It is stable-baselines3's old default and the
  weights were trained with it; the modern default of ``ReLU`` is the wrong
  function for this checkpoint.
- the BatchNorm layers run in ``reference`` mode, that is in *training* mode.
  ``stable_baselines3`` 1.1's ``predict`` never switched the policy to inference
  mode, so the published deployment normalised with the statistics of the single
  sample it was given. ``running`` mode uses the accumulated running statistics
  instead, which is what the layer was meant to do. The default matches what the
  published code actually did.
"""

from __future__ import annotations

import io
import pathlib
import sys
import zipfile

import numpy as np

# ``custom_cnn_full`` imports ``gym``, which this host does not ship; the vendored
# shim answers to that name so the import works and the pickled observation space
# inside the checkpoint still resolves.
_VENDOR = pathlib.Path(__file__).resolve().parent / "vendor"
if str(_VENDOR) not in sys.path:
    sys.path.insert(0, str(_VENDOR))

import torch  # noqa: E402
from torch import nn  # noqa: E402

import custom_cnn_full  # noqa: E402  (vendored)

#: Width of the feature vector both heads read.
FEATURES_DIM = 256
#: Length of the observation the policy takes: 12800 ped + 6400 scan + 2 goal.
OBSERVATION_DIM = 12800 + 6400 + 2
#: The policy outputs one forward speed and one yaw rate, both in [-1, 1].
ACTION_DIM = 2


def _load_state_dict(path: pathlib.Path) -> dict:
    """The tensors of ``policy.pth`` inside a stable-baselines3 checkpoint."""
    with zipfile.ZipFile(path) as archive:
        blob = archive.read("policy.pth")
    try:
        return torch.load(io.BytesIO(blob), map_location="cpu", weights_only=False)
    except TypeError:  # torch < 1.13 spells the argument differently
        return torch.load(io.BytesIO(blob), map_location="cpu")


class DrlVoPolicy:
    """The DRL-VO control policy, ready to turn an observation into an action."""

    def __init__(self, checkpoint, batchnorm: str = "reference"):
        if batchnorm not in ("reference", "running"):
            raise ValueError(f"batchnorm must be 'reference' or 'running', not {batchnorm!r}")

        path = pathlib.Path(checkpoint)
        if not path.is_file():
            raise FileNotFoundError(f"no DRL-VO checkpoint at {path}")

        parameters = _load_state_dict(path)
        state = {key: value for key, value in parameters.items() if key.startswith("features_extractor.")}

        space = _observation_space()
        self.features = custom_cnn_full.CustomCNN(space, features_dim=FEATURES_DIM)
        self.features.load_state_dict(
            {key[len("features_extractor."):]: value for key, value in state.items()}
        )
        self.policy_net = nn.Sequential(
            nn.Linear(FEATURES_DIM, parameters["mlp_extractor.policy_net.0.weight"].shape[0]),
            nn.Tanh(),
        )
        self.policy_net[0].weight.data = parameters["mlp_extractor.policy_net.0.weight"]
        self.policy_net[0].bias.data = parameters["mlp_extractor.policy_net.0.bias"]
        self.action_net = nn.Linear(FEATURES_DIM, ACTION_DIM)
        self.action_net.weight.data = parameters["action_net.weight"]
        self.action_net.bias.data = parameters["action_net.bias"]

        # ``reference`` keeps the published behaviour; ``running`` is what the
        # BatchNorm layers were meant to do at deployment.
        training = batchnorm == "reference"
        self.features.train(training)
        self.policy_net.train(training)
        self.action_net.train(training)
        if training:
            # Training mode recomputes the batch statistics *and* folds them into
            # the running ones. Only the first half is wanted here - the forward
            # pass is the published one either way - so momentum is zeroed and the
            # checkpoint's running statistics stay as they were saved.
            for module in self.features.modules():
                if isinstance(module, nn.modules.batchnorm._BatchNorm):
                    module.momentum = 0.0
                    if module.num_batches_tracked is not None:
                        module.num_batches_tracked.fill_(0)

    def predict(self, observation) -> np.ndarray:
        """The deterministic action, ``float32[2]``, clipped to ``[-1, 1]``.

        The policy is a Gaussian whose deterministic action is its mean, and
        stable-baselines3's ``predict(..., deterministic=True)`` clipped that
        mean to the action space. The clamp here is that clip.
        """
        tensor = torch.as_tensor(np.asarray(observation, dtype=np.float32)).reshape(1, -1)
        with torch.no_grad():
            mean = self.action_net(self.policy_net(self.features(tensor)))
        return np.clip(mean.numpy().reshape(-1), -1.0, 1.0).astype(np.float32)


def _observation_space():
    """The flat ``Box(-1, 1, (19202,))`` the checkpoint was trained with."""
    import gymnasium

    return gymnasium.spaces.Box(-1.0, 1.0, shape=(OBSERVATION_DIM,), dtype=np.float32)
