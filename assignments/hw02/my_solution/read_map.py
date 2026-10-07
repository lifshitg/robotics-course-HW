#!/usr/bin/env python3
"""Receive and save the supplied raw map. Does not construct configuration space."""
from pathlib import Path
import time
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from hw02_sim.map_server import MAP_QOS


def main():
    rclpy.init(); node = rclpy.create_node('hw02_read_map'); messages = []
    # Transient-local durability receives the map even when subscribing late.
    node.create_subscription(OccupancyGrid, '/map', messages.append, MAP_QOS)
    try:
        deadline = time.monotonic()+20
        while not messages and rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not messages:
            print('FAIL: no map; check simulation, sourcing, domain and map QoS'); return 1
        message = messages[-1]; info = message.info
        grid = np.asarray(message.data, dtype=np.int8).reshape(info.height,info.width)
        print(f'Frame={message.header.frame_id}; size={info.width}x{info.height}; resolution={info.resolution:.3f} m')
        print(f'Origin=({info.origin.position.x}, {info.origin.position.y}); occupied={np.count_nonzero(grid==100)}; unknown={np.count_nonzero(grid<0)}')
        output = Path(__file__).resolve().parents[3]/'build/hw02/demo_map'
        output.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(output/'raw_map.npz', data=grid, resolution=info.resolution,
                            origin=[info.origin.position.x,info.origin.position.y], frame=message.header.frame_id)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        x, y = info.origin.position.x, info.origin.position.y
        plt.imshow(grid, origin='lower', cmap='gray_r', vmin=0, vmax=100,
                   extent=(x,x+info.width*info.resolution,y,y+info.height*info.resolution))
        plt.xlabel('map X (m)'); plt.ylabel('map Y (m)'); plt.title('Supplied physical occupancy map')
        plt.savefig(output/'raw_map.png',dpi=160); plt.close()
        print(f'PASS: map received and saved to {output}'); return 0
    except (KeyboardInterrupt, ValueError): return 1
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': raise SystemExit(main())
