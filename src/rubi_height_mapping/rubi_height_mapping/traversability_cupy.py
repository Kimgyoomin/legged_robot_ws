# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under MIT; see LICENSE and UPSTREAM.md.
"""CuPy equivalent of the inherited four-convolution inference.

This internal score feeds the height estimator's inlier/wall accounting. It is
not a published walking-safety layer. Keep it to preserve that coupling without
requiring PyTorch, Chainer, cuDNN or loading pickled model objects.
"""
import numpy as np
import cupy as cp


class CupyTraversability:
    def __init__(self, weights_path):
        with np.load(weights_path, allow_pickle=False) as weights:
            blocks = [weights[k] for k in ('w1', 'w2', 'w3')]
            out = weights['w_out']
            if any(a.shape != (4, 1, 3, 3) for a in blocks) or out.shape != (1, 12, 1, 1):
                raise ValueError('Unexpected fixed-filter weight shape')
            if not all(np.isfinite(a).all() for a in blocks + [out]):
                raise ValueError('Nonfinite filter weights')
            self.weights = cp.asarray(np.concatenate([a.ravel() for a in blocks]), dtype=cp.float32)
            self.output_weights = cp.asarray(out.ravel(), dtype=cp.float32)
        self.kernel = cp.ElementwiseKernel(
            'raw float32 image, raw float32 weights, raw float32 output_weights, int32 width',
            'float32 score',
            r'''
            int out_width = width - 6;
            int y = i / out_width + 3;
            int x = i % out_width + 3;
            float total = 0.0f;
            for (int b = 0; b < 3; ++b) {
                int dilation = b + 1;
                for (int c = 0; c < 4; ++c) {
                    float response = 0.0f;
                    for (int ky = -1; ky <= 1; ++ky)
                        for (int kx = -1; kx <= 1; ++kx)
                            response += image[(y + ky*dilation)*width + x + kx*dilation]
                                * weights[b*36 + c*9 + (ky+1)*3 + kx+1];
                    total += fabsf(response) * output_weights[b*4+c];
                }
            }
            score = expf(-total);
            ''', 'rubi_cupy_traversability')

    def __call__(self, elevation):
        if elevation.ndim != 2 or elevation.shape[0] != elevation.shape[1] or elevation.shape[0] < 7:
            raise ValueError('Expected square height buffer at least 7 x 7')
        size = elevation.shape[0] - 6
        result = cp.empty((size, size), dtype=cp.float32)
        self.kernel(cp.ascontiguousarray(elevation, dtype=cp.float32), self.weights,
                    self.output_weights, np.int32(elevation.shape[1]), result)
        return result.reshape(1, 1, size, size)
