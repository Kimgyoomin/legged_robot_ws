# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under MIT; see LICENSE and UPSTREAM.md.
# RUBI height-only derivative, 2026-10-14.
from array import array as FloatArray
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayLayout as MAL, MultiArrayDimension as MAD

def encode_layer_to_multiarray(array: np.ndarray, layout: str = "gridmap_column") -> Float32MultiArray:
    """Encode a (rows, cols) array into a Float32MultiArray."""
    arr = np.asarray(array, dtype=np.float32)
    rows, cols = arr.shape
    msg = Float32MultiArray()
    msg.layout = MAL()

    if layout == "gridmap_column":
        msg.layout.dim.append(MAD(label="column_index", size=cols, stride=rows * cols))
        msg.layout.dim.append(MAD(label="row_index", size=rows, stride=rows))
        payload = FloatArray("f")
        payload.frombytes(arr.tobytes(order="F"))
        msg.data = payload
        return msg

    if layout == "row_major":
        msg.layout.dim.append(MAD(label="row_index", size=rows, stride=rows * cols))
        msg.layout.dim.append(MAD(label="column_index", size=cols, stride=cols))
        payload = FloatArray("f")
        payload.frombytes(arr.tobytes(order="C"))
        msg.data = payload
        return msg

    raise ValueError(f"Unknown layout '{layout}'")
