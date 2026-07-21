#!/usr/bin/env python3
"""Feed ROS1/ROS2 IMU and PointCloud2 messages into Wildcat SLAM."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


POINT_FIELD_TYPES = {
    1: "i1",  # INT8
    2: "u1",  # UINT8
    3: "i2",  # INT16
    4: "u2",  # UINT16
    5: "i4",  # INT32
    6: "u4",  # UINT32
    7: "f4",  # FLOAT32
    8: "f8",  # FLOAT64
}


class RerunVisualizer:
    def __init__(self) -> None:
        try:
            import rerun as rr
        except ImportError as error:
            raise RuntimeError(
                "Visualization requires the optional dependency: run `uv sync --extra visualization`"
            ) from error

        self.rr = rr
        self.trajectory: list[np.ndarray] = []
        rr.init("wildcat_slam", spawn=True)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    def log_snapshot(self, sweep_count: int, snapshot: dict[str, object]) -> None:
        rr = self.rr
        rr.set_time("sweep", sequence=sweep_count)
        rr.set_time("sensor_time", timestamp=float(snapshot["timestamp"]))

        position = np.asarray(snapshot["position"])
        orientation = np.asarray(snapshot["orientation_xyzw"])
        rr.log(
            "world/imu",
            rr.Transform3D(
                translation=position,
                quaternion=rr.Quaternion(xyzw=orientation),
            ),
            rr.TransformAxes3D(0.5),
        )

        self.trajectory.append(position.copy())
        rr.log(
            "world/trajectory",
            rr.LineStrips3D([np.asarray(self.trajectory)], colors=[255, 220, 80]),
        )

        surfels = snapshot["surfels"]
        centers = np.asarray(surfels["centers"])
        normals = np.asarray(surfels["normals"])
        fixed_window = np.asarray(surfels["fixed_window"], dtype=bool)
        colors = np.empty((len(centers), 3), dtype=np.uint8)
        colors[~fixed_window] = [70, 160, 255]
        colors[fixed_window] = [255, 150, 60]
        rr.log("world/surfels/centers", rr.Points3D(centers, colors=colors, radii=0.025))
        rr.log(
            "world/surfels/normals",
            rr.Arrows3D(origins=centers, vectors=0.25 * normals, colors=colors),
        )


def stamp_seconds(stamp: object) -> float:
    nanoseconds = getattr(stamp, "nanosec", getattr(stamp, "nsec", 0))
    return float(stamp.sec) + float(nanoseconds) * 1e-9


def pointcloud_array(message: object) -> np.ndarray:
    # PointCloud2 stores each point as a binary record. Build a structured
    # NumPy dtype from the message's field table so those records can be viewed
    # by field name (for example, points["x"] or points["timestamp"]).
    endian = ">" if message.is_bigendian else "<"
    names: list[str] = []
    formats: list[object] = []
    offsets: list[int] = []
    for field in message.fields:
        if field.datatype not in POINT_FIELD_TYPES:
            raise ValueError(f"Unsupported PointField datatype {field.datatype} for {field.name}")
        names.append(field.name)
        # Preserve both the message byte order and fixed-size array fields.
        scalar = np.dtype(endian + POINT_FIELD_TYPES[field.datatype])
        formats.append(scalar if field.count == 1 else (scalar, (field.count,)))
        offsets.append(field.offset)

    dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": message.point_step}
    )

    # A tightly packed cloud can be exposed as a zero-copy view over the bag
    # message's byte buffer.
    data = memoryview(message.data)
    packed_row_size = message.width * message.point_step
    if message.row_step == packed_row_size:
        return np.frombuffer(data, dtype=dtype, count=message.width * message.height)

    # Organized clouds may add padding at the end of every image row. Strip
    # that padding before interpreting the remaining bytes as point records.
    rows = [
        data[row * message.row_step : row * message.row_step + packed_row_size]
        for row in range(message.height)
    ]
    return np.frombuffer(b"".join(rows), dtype=dtype, count=message.width * message.height)


def point_times(points: np.ndarray, header_time: float, field_name: str, mode: str) -> np.ndarray:
    if field_name not in points.dtype.names:
        available = ", ".join(points.dtype.names or ())
        raise ValueError(f"Point time field {field_name!r} is missing; available fields: {available}")
    times = np.asarray(points[field_name], dtype=np.float64)
    if mode == "relative" or (mode == "auto" and times.size and np.nanmax(np.abs(times)) < 1e6):
        times = times + header_time
    return times


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--imu-topic", default="/alphasense/imu")
    parser.add_argument("--lidar-topic", default="/hesai/pandar")
    parser.add_argument("--imu-rate", type=int, default=200)
    parser.add_argument("--point-time-field", default="timestamp")
    parser.add_argument("--point-time-mode", choices=("auto", "absolute", "relative"), default="auto")
    parser.add_argument("--visualize", action="store_true", help="Stream poses and surfels to Rerun")
    args = parser.parse_args()

    import wildcat_slam  # pylint: disable=import-error,import-outside-toplevel

    processor = wildcat_slam.Processor(args.imu_rate)
    visualizer = RerunVisualizer() if args.visualize else None
    wanted_topics = {args.imu_topic, args.lidar_topic}
    with AnyReader([args.bag]) as reader:
        connections = [connection for connection in reader.connections if connection.topic in wanted_topics]
        found_topics = {connection.topic for connection in connections}
        missing = wanted_topics - found_topics
        if missing:
            raise RuntimeError(f"Topics not found in bag: {', '.join(sorted(missing))}")

        for connection, _, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == args.imu_topic:
                processor.add_imu(
                    stamp_seconds(message.header.stamp),
                    np.array(
                        [message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z]
                    ),
                    np.array([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z]),
                )
            else:
                lidar_time = stamp_seconds(message.header.stamp)
                points = pointcloud_array(message)
                required = {"x", "y", "z"}
                if not required.issubset(points.dtype.names or ()):
                    raise ValueError(f"Point cloud is missing fields: {sorted(required - set(points.dtype.names or ())) }")
                count = len(points)
                if count == 0:
                    continue
                xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(np.float32)
                intensity = np.asarray(points["intensity"], dtype=np.float32) if "intensity" in points.dtype.names else np.zeros(count, np.float32)
                ring = np.asarray(points["ring"], dtype=np.uint16) if "ring" in points.dtype.names else np.zeros(count, np.uint16)
                times = point_times(
                    points,
                    lidar_time,
                    args.point_time_field,
                    args.point_time_mode,
                )
                previous_sweep_count = processor.sweep_count
                processor.add_lidar(xyz, intensity, times, ring)
                sweep_completed = processor.sweep_count > previous_sweep_count
                if not sweep_completed:
                    continue

                latest_pose = processor.latest_pose
                if latest_pose is None:
                    print(f"time={lidar_time:.9f} position=unavailable")
                else:
                    x, y, z = latest_pose["position"]
                    print(f"time={lidar_time:.9f} position=({x:.3f}, {y:.3f}, {z:.3f})")
                timings = processor.timing_averages
                print(
                    "running average timings: "
                    + ", ".join(f"{stage}={seconds * 1000:.3f} ms" for stage, seconds in timings.items())
                )
                if visualizer is not None:
                    snapshot = processor.snapshot()
                    if snapshot is not None:
                        visualizer.log_snapshot(processor.sweep_count, snapshot)

    print(f"Processed {processor.sweep_count} sweeps")
    print(f"Latest pose: {processor.latest_pose}")
