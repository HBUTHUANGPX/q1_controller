#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path


def load_proto_module(proto_root: Path):
    sys.path.insert(0, str(proto_root))
    import link_states_pb2  # type: ignore

    return link_states_pb2


def main():
    parser = argparse.ArgumentParser(
        description="Receive xsens /link_states protobuf payloads over ZMQ PUB/SUB.")
    parser.add_argument(
        "--connect",
        default="tcp://127.0.0.1:5555",
        help="ZMQ address to connect to, for example tcp://192.168.1.10:5555",
    )
    parser.add_argument(
        "--topic",
        default="xsens.link_states.v1",
        help="ZMQ subscription topic prefix",
    )
    parser.add_argument(
        "--proto-python-dir",
        default=None,
        help="Directory containing generated link_states_pb2.py",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of protobuf messages to print before exiting",
    )
    args = parser.parse_args()

    try:
        import zmq
    except ImportError as exc:
        raise SystemExit("Missing pyzmq. Install with: pip install pyzmq") from exc

    if args.proto_python_dir is None:
        raise SystemExit(
            "--proto-python-dir is required. Generate it with:\n"
            "  protoc --proto_path src/xsens_mvn_ros2/proto "
            "--python_out /tmp/xsens_proto src/xsens_mvn_ros2/proto/link_states.proto"
        )

    proto_dir = Path(args.proto_python_dir).expanduser().resolve()
    if not proto_dir.exists():
        raise SystemExit(f"Proto python directory does not exist: {proto_dir}")

    link_states_pb2 = load_proto_module(proto_dir)

    context = zmq.Context.instance()
    socket = context.socket(zmq.SUB)
    socket.setsockopt_string(zmq.SUBSCRIBE, args.topic)
    socket.connect(args.connect)

    for index in range(args.count):
        topic = socket.recv_string()
        payload = socket.recv()

        proto_msg = link_states_pb2.LinkStateArray()
        proto_msg.ParseFromString(payload)

        print(f"message_index={index}")
        print(f"topic={topic}")
        print(f"schema_version={proto_msg.header.schema_version}")
        print(f"frame_id={proto_msg.header.frame_id}")
        print(f"states_size={len(proto_msg.states)}")

        for state_idx, state in enumerate(proto_msg.states):
            print(
                f"state[{state_idx}].name={state.name} "
                f"position={state.pose.position.x},{state.pose.position.y},{state.pose.position.z}"
            )


if __name__ == "__main__":
    main()
