#
# Sample Python client for the AI GP controller
#

import time

from setup import setup_components


##############
import logging
import os

# Configuration du fichier de log
os.makedirs('log', exist_ok=True)

logging.basicConfig(
    filename='log/flight_log.txt',
    filemode='w',  # 'w' pour écraser le fichier à chaque nouveau lancement
    format='%(asctime)s [%(levelname)s] %(message)s',
    encoding='utf8',
    datefmt='%H:%M:%S',
    level=logging.INFO
)

logger = logging.getLogger("DronePilot")
logger.info("--- DÉMARRAGE DU LOG DE VOL ---")
##############




# Modify these properties if you want to run the server remotely for example
SIM_SERVER_UDP_IP = "127.0.0.1"
SIM_SERVER_UDP_PORT = 14550

# time since sim started ms
system_boot_ms = int(time.time() * 1000)

# arbitrary shared data between the various components
shared_data = {}

# setup components
components = setup_components(shared_data, system_boot_ms, SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT)
controller = components['controller']
ts_loop = components['ts_loop']
mavlink_rx = components['mavlink_rx']
vision_rx = components['vision_rx']

print("Arming drone...", flush=True)
controller.arm()
print("Starting control loop...", flush=True)
is_running = True
try:
    while is_running:
        controller.update()

except KeyboardInterrupt:
    # exit
    ts_loop.get_thread_for_join().join(timeout=1.0)#TODO problème à régler NoneType has no attribute
    mavlink_rx.get_thread_for_join().join(timeout=1.0)
    vision_rx.get_thread_for_join().join(timeout=1.0)
finally :
    print("Client exited!", flush=True)
