from dataclasses import dataclass
from multiprocessing import Process

import tyro

from gello.cameras.realsense_camera import RealSenseCamera, get_device_ids
from gello.cameras.zed_camera import Zed2iCamera
from gello.zmq_core.camera_node import ZMQServerCamera


@dataclass
class Args:
    hostname: str = "127.0.0.1"


def launch_server(port: int, camera_brand:str, camera_id, args: Args):
    if camera_brand == "zed":
        camera = Zed2iCamera(camera_id)
    elif camera_brand == "realsense":
        camera = RealSenseCamera(camera_id)
    else: 
        raise ValueError(f"camera brand {camera_brand} unknown")
    
    server = ZMQServerCamera(camera, port=port, host=args.hostname)
    print(f"Starting camera server on port {port}")
    server.serve()


camera_configs = [
    # (brand, serial number, port ID)
   ("realsense", "944122073290", 5000), # left wrist camera
   #("realsense","817612070315",5001), # right wrist camera,
   ("zed", 30209878, 5002), # base camera
     
]
def main(args):
    ids = get_device_ids()
    camera_servers = []
    for config in camera_configs:
        camera_brand, camera_id, camera_port = config


        print(f"Launching camera {camera_brand}-{camera_id} on port {camera_port}")
        camera_servers.append(
            Process(target=launch_server, args=(camera_port, camera_brand, camera_id, args))
        )
       

    for server in camera_servers:
        server.start()

    # Check whether all processes are still alive after a few seconds to prevent silent errors in camera startup
    import time
    time.sleep(10)

    for server in camera_servers:
        if not server.is_alive():
            raise RuntimeError("A camera server has died (check connection, serial number, etc.)!")


if __name__ == "__main__":
    main(tyro.cli(Args))
