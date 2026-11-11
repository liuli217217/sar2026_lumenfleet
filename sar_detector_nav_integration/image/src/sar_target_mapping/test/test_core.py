"""DETECTOR_NAV_INTEGRATION: synthetic geometry tests, no ROS/GPU required."""
import ast
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from sar_target_mapping.core import depth_point, transform_point, associate


def inputs(encoding='16UC1', big=False, padded=False):
    width = height = 20
    size, fmt, value = (2, 'H', 2000) if encoding == '16UC1' else (4, 'f', 2.)
    padding = b'\x00'*8 if padded else b''
    data = (struct.pack(('>' if big else '<')+fmt, value)*width+padding)*height
    image = NS(width=width, height=height, encoding=encoding, is_bigendian=big,
               step=width*size+len(padding), data=data)
    info = NS(width=width, height=height, k=[10., 0., 10., 0., 10., 10., 0., 0., 1.])
    return image, info


class GeometryTests(unittest.TestCase):
    def test_depth_encodings_endian_padding(self):
        for encoding in ('16UC1', '32FC1'):
            for big in (False, True):
                image, info = inputs(encoding, big, True)
                self.assertEqual(depth_point(image, info, (10, 10, 10, 10)), (0., 0., 2.))
                self.assertEqual(depth_point(image, info, (15, 10, 10, 10)), (1., 0., 2.))

    def test_invalid_depth_rejected(self):
        image, info = inputs()
        image.data = bytes(len(image.data))
        with self.assertRaises(ValueError):
            depth_point(image, info, (10, 10, 10, 10))
        image, info = inputs()
        image.data = image.data[:5]
        with self.assertRaises(ValueError):
            depth_point(image, info, (10, 10, 10, 10))

    def test_bad_intrinsics_and_box(self):
        image, info = inputs()
        info.k[0] = 0
        with self.assertRaises(ValueError):
            depth_point(image, info, (10, 10, 10, 10))
        image, info = inputs()
        with self.assertRaises(ValueError):
            depth_point(image, info, (float('nan'), 10, 10, 10))

    def test_rotation_translation(self):
        result = transform_point((1, 0, 0), (1, 2, 3), (0, 0, math.sqrt(.5), math.sqrt(.5)))
        for actual, expected in zip(result, (1, 3, 3)):
            self.assertAlmostEqual(actual, expected)
        with self.assertRaises(ValueError):
            transform_point((1, 0, 0), (0, 0, 0), (0, 0, 0, 0))

    def test_spatial_dedup_does_not_use_track_id(self):
        records = [dict(class_id='mannequin', position=[1, 2, 0]),
                   dict(class_id='cone', position=[1, 2, 0])]
        self.assertEqual(associate(records, 'mannequin', (1.1, 2, 1), .6), 0)
        self.assertEqual(associate(records, 'cone', (1.1, 2, 1), .6), 1)
        self.assertIsNone(associate(records, 'mannequin', (3, 2, 0), .6))

    def test_per_class_threshold_reaches_predict(self):
        # Execute the real copied detector initializer with lightweight ROS/model stubs.
        source = PACKAGE.parent/'victim_detector_integrated/victim_detector_integrated/detector_node.py'
        tree = ast.parse(source.read_text())
        wanted = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
        class FakeNode:
            def __init__(self, name):
                self.params = {}
            def declare_parameter(self, name, value):
                self.params[name] = {'class_conf': 'mannequin:0.10,cone:0.20'}.get(name, value)
            def get_parameter(self, name):
                return NS(value=self.params[name])
            def __getattr__(self, name):
                if name == 'get_logger':
                    return lambda: NS(info=lambda *a: None)
                return lambda *a, **kw: None
        ns = dict(Node=FakeNode, CvBridge=lambda: None, Image=object,
                  Detection2DArray=object, String=object)
        exec(compile(ast.Module(body=wanted, type_ignores=[]), '<detector>', 'exec'), ns)
        ns['sensor_qos'] = lambda *args: None
        cls = ns['SarDetector']
        cls._load_model = lambda *a: (None, ['cone', 'first_aid', 'mannequin'], 'cpu')
        detector = cls()
        self.assertEqual(detector.conf, .25)
        self.assertEqual(detector.inference_conf, .10)
        # Verify the actual on_image method sends the lowered threshold into YOLO.
        calls = []
        detector.model = NS(predict=lambda *a, **kw: (calls.append(kw) or [NS(boxes=[])]))
        detector.bridge = NS(imgmsg_to_cv2=lambda *a, **kw: None)
        detector.pub_det = NS(publish=lambda x: None)
        detector.pub_img = None
        detector._to_msg = lambda *a: None
        ns['time'] = NS(perf_counter=iter([10., 10., 10.05]).__next__)
        detector.on_image(NS(header=None))
        self.assertEqual(calls[0]['conf'], .10)


if __name__ == '__main__':
    unittest.main()
