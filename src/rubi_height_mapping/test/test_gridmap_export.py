"""The optimized serializer must preserve the ROS wire representation."""
from array import array
import numpy as np
from rclpy.serialization import serialize_message
from rubi_height_mapping.gridmap_utils import encode_layer_to_multiarray


def test_float_buffer_matches_legacy_list_on_wire():
    source = np.arange(35, dtype=np.float32).reshape(5, 7)
    source[1, 2] = np.nan
    source[3, 4] = np.inf
    source[0, 0] = -0.0
    for data in (source, source.T, source[::-1, ::2]):
        for layout, order in (("gridmap_column", "F"), ("row_major", "C")):
            actual = encode_layer_to_multiarray(data, layout)
            expected = encode_layer_to_multiarray(data, layout)
            expected.data = data.flatten(order=order).tolist()
            assert isinstance(actual.data, array)
            assert serialize_message(actual) == serialize_message(expected)
