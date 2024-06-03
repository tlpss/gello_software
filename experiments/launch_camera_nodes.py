from dataclasses import dataclass
from multiprocessing import Process

import tyro

from gello.cameras.realsense_camera import RealSenseCamera, get_device_ids
from gello.zmq_core.camera_node import ZMQServerCamera


@dataclass
class Args:
    hostname: str = "127.0.0.1"


def launch_server(port: int, camera_id: int, args: Args):
    camera = RealSenseCamera(camera_id)
    server = ZMQServerCamera(camera, port=port, host=args.hostname)
    print(f"Starting camera server on port {port}")
    server.serve()


# fix the mapping instead of having it depend on the # of cameras connected.

id_to_port_mapping = {
    "925322060348" : 5000, # wrist 
    "943222073454": 5001, # base
}
def main(args):
    ids = get_device_ids()
    camera_servers = []
    for camera_id in ids:
        try:
            camera_port = id_to_port_mapping[camera_id]
        except KeyError:
            raise ValueError(f"unknown camera id: {camera_id}")
        # start a python process for each camera
        print(f"Launching camera {camera_id} on port {camera_port}")
        camera_servers.append(
            Process(target=launch_server, args=(camera_port, camera_id, args))
        )

    for server in camera_servers:
        server.start()


if __name__ == "__main__":
    main(tyro.cli(Args))
