"""Minimal MLP (NumPy) for PPO / MAPPO actors and critics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np


def _xavier(fan_in: int, fan_out: int, rng: np.random.Generator) -> np.ndarray:
    lim = np.sqrt(6.0 / (fan_in + fan_out))
    return rng.uniform(-lim, lim, size=(fan_in, fan_out)).astype(np.float64)


@dataclass
class Layer:
    w: np.ndarray
    b: np.ndarray


class MLP:
    """tanh hidden layers → linear output."""

    def __init__(self, sizes: Sequence[int], rng: np.random.Generator | None = None) -> None:
        self.rng = rng or np.random.default_rng(0)
        self.layers: List[Layer] = []
        for i in range(len(sizes) - 1):
            self.layers.append(Layer(
                w=_xavier(sizes[i], sizes[i + 1], self.rng),
                b=np.zeros(sizes[i + 1], dtype=np.float64),
            ))

    def forward(self, x: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        """Return (out, pre-activations including input)."""
        acts = [x]
        h = x
        for i, layer in enumerate(self.layers):
            z = h @ layer.w + layer.b
            if i < len(self.layers) - 1:
                h = np.tanh(z)
            else:
                h = z
            acts.append(h)
        return h, acts

    def parameters(self) -> List[np.ndarray]:
        params: List[np.ndarray] = []
        for layer in self.layers:
            params.extend([layer.w, layer.b])
        return params

    def set_parameters(self, params: Sequence[np.ndarray]) -> None:
        idx = 0
        for layer in self.layers:
            layer.w = np.array(params[idx], dtype=np.float64, copy=True)
            layer.b = np.array(params[idx + 1], dtype=np.float64, copy=True)
            idx += 2

    def zero_grad_like(self) -> List[np.ndarray]:
        return [np.zeros_like(p) for p in self.parameters()]

    def backward(
        self,
        acts: List[np.ndarray],
        dout: np.ndarray,
    ) -> List[np.ndarray]:
        """Backprop; dout is ∂L/∂output. Returns grads matching parameters()."""
        grads: List[np.ndarray] = []
        g = dout
        for i in reversed(range(len(self.layers))):
            h_prev = acts[i]
            if i < len(self.layers) - 1:
                # acts[i+1] = tanh(z); g is ∂L/∂h
                g = g * (1.0 - acts[i + 1] ** 2)
            dw = h_prev.T @ g if h_prev.ndim == 2 else np.outer(h_prev, g)
            db = g.sum(axis=0) if g.ndim == 2 else g
            grads.append(db)
            grads.append(dw)
            g = g @ self.layers[i].w.T if g.ndim == 2 else self.layers[i].w @ g
        grads.reverse()
        # parameters order is w,b,w,b,...; we appended db,dw then reverse → w,b order? 
        # After reverse: for each layer from first: we had append db, dw then reverse whole list
        # Layer0: ... let's rebuild carefully
        return self._reorder_grads(acts, dout)

    def _reorder_grads(self, acts: List[np.ndarray], dout: np.ndarray) -> List[np.ndarray]:
        grads = self.zero_grad_like()
        g = dout
        param_i = len(grads) - 1
        for i in reversed(range(len(self.layers))):
            h_prev = acts[i]
            if i < len(self.layers) - 1:
                g = g * (1.0 - acts[i + 1] ** 2)
            if g.ndim == 1:
                dw = np.outer(h_prev, g)
                db = g.copy()
                g_prev = self.layers[i].w @ g
            else:
                dw = h_prev.T @ g
                db = g.sum(axis=0)
                g_prev = g @ self.layers[i].w.T
            # params: [w0,b0,w1,b1,...]
            wi = 2 * i
            grads[wi] = dw
            grads[wi + 1] = db
            g = g_prev
            _ = param_i
        return grads

    def apply_grads(self, grads: Sequence[np.ndarray], lr: float, max_norm: float = 1.0) -> None:
        flat = np.concatenate([g.ravel() for g in grads])
        n = float(np.linalg.norm(flat)) + 1e-8
        scale = min(1.0, max_norm / n)
        for p, g in zip(self.parameters(), grads):
            p -= lr * scale * g

    def state_dict(self) -> dict:
        return {'params': [p.copy() for p in self.parameters()],
                'sizes': [self.layers[0].w.shape[0]] + [l.b.shape[0] for l in self.layers]}

    @classmethod
    def from_state_dict(cls, sd: dict, rng: np.random.Generator | None = None) -> 'MLP':
        net = cls(sd['sizes'], rng=rng)
        net.set_parameters(sd['params'])
        return net
