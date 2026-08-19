"""Shared helpers for the offline LIO-SAM bag converters.

rosbags reads ROS 1 bags without a ROS 1 installation and normalises type names
to the ROS 2 form ('sensor_msgs/msg/Imu'), so one code path serves both stages.
Messages still have to be rebuilt as ROS 2 dataclasses before they can be
serialised into a ROS 2 bag: the classes are distinct either way, and ROS 1's
Header carries a 'seq' field that ROS 2 dropped.
"""
from __future__ import annotations

import dataclasses

import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_JAZZY)

# PointCloud2 field datatype -> numpy format.
_PF_DTYPE = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def msg_class(typename):
    return TYPESTORE.types[typename]


def _init_fields(cls):
    """Data fields only.

    rosbags renders message constants (PointField.INT8, NavSatStatus.STATUS_FIX)
    as ordinary dataclass fields with defaults, and the sets differ between ROS 1
    and ROS 2. They are fixed by the type, so never copy them - ROS field names
    are lower_snake_case, so upper-case means constant.
    """
    return [f.name for f in dataclasses.fields(cls)
            if f.name != '__msgtype__' and not f.name.isupper()]


def to_ros2(src, typename=None):
    """Rebuild a message as its ROS 2 dataclass, recursively.

    Iterating the *destination* fields rather than the source is what silently
    drops ROS 1-only fields such as Header.seq.
    """
    typename = typename or src.__msgtype__
    cls = TYPESTORE.types[typename]
    return cls(**{name: _convert(getattr(src, name))
                  for name in _init_fields(cls) if hasattr(src, name)})


def _convert(val):
    if hasattr(val, '__msgtype__'):
        return to_ros2(val)
    if isinstance(val, np.ndarray):
        return val                      # numeric arrays need no rebuilding
    if isinstance(val, (list, tuple)):
        return [_convert(v) for v in val]
    return val


def time_msg(sec, nanosec):
    return TYPESTORE.types['builtin_interfaces/msg/Time'](sec=int(sec), nanosec=int(nanosec))


def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def ns_to_stamp(ns):
    ns = int(ns)
    return time_msg(ns // 1_000_000_000, ns % 1_000_000_000)


def cloud_to_array(msg):
    """View a PointCloud2's payload as a structured array, padding included."""
    dt = np.dtype({
        'names': [f.name for f in msg.fields],
        'formats': [_PF_DTYPE[f.datatype] for f in msg.fields],
        'offsets': [f.offset for f in msg.fields],
        'itemsize': msg.point_step,
    })
    return np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)


def array_to_cloud(arr, header, fields, point_step):
    """Build an unorganised (height=1) PointCloud2 from a structured array."""
    PointCloud2 = TYPESTORE.types['sensor_msgs/msg/PointCloud2']
    return PointCloud2(
        header=header,
        height=1,
        width=len(arr),
        fields=list(fields),
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * len(arr),
        data=np.frombuffer(arr.tobytes(), dtype=np.uint8).copy(),
        is_dense=True,
    )


class Bag2Writer:
    """Writes a ROS 2 (mcap) bag, creating one connection per topic on demand."""

    def __init__(self, path):
        self._writer = Writer(path, version=9, storage_plugin=StoragePlugin.MCAP)
        self._connections = {}
        self.counts = {}

    def __enter__(self):
        self._writer.open()
        return self

    def __exit__(self, *exc):
        self._writer.close()

    def write(self, topic, msg, timestamp_ns, typename=None):
        typename = typename or msg.__msgtype__
        conn = self._connections.get(topic)
        if conn is None:
            conn = self._writer.add_connection(topic, typename, typestore=TYPESTORE)
            self._connections[topic] = conn
        self._writer.write(conn, int(timestamp_ns),
                           TYPESTORE.serialize_cdr(msg, typename))
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def report(self):
        width = max((len(t) for t in self.counts), default=0)
        for topic in sorted(self.counts):
            print(f'  {topic:<{width}}  {self.counts[topic]}')


def progress(iterable, **kwargs):
    """tqdm when available, a plain pass-through otherwise."""
    try:
        from tqdm import tqdm
    except ImportError:
        return iterable
    return tqdm(iterable, **kwargs)
