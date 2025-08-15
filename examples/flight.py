###################################
# IMPORTS

# Add parent directory to sys.path so that we can import from ae483clients
import sys, os
sys.path.append(os.path.abspath('..'))
from ae483clients import CrazyflieClient, QualisysClient

# Do all other imports
import time
import json


###################################
# PARAMETERS

# -- PROBABLY THE SAME FOR EVERY FLIGHT IN LABS 1-10 --

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Specify the name of the rigid body that corresponds to your active marker
# deck in the motion capture system. If your marker deck number is X, this name
# should be 'marker_deck_X'.
marker_deck_name = 'marker_deck_90'

# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
marker_deck_ids = [91, 92, 93, 94]

# -- MAY CHANGE FROM FLIGHT TO FLIGHT --

# Specify whether or not to use the motion capture system
use_mocap = True

# Specify whether or not to use a custom controller
use_controller = False

# Specify whether or not to use a custom observer
use_observer = False

# Specify the variables you want to log at 100 Hz from the drone
variables = [
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z',
    'stateEstimate.yaw',
    'stateEstimate.pitch',
    'stateEstimate.roll',
]


###################################
# FLIGHT CODE

# Create and start the client that will connect to the drone
drone_client = CrazyflieClient(
    uri,
    use_controller=use_controller,
    use_observer=use_observer,
    marker_deck_ids=marker_deck_ids if use_mocap else None,
    variables=variables,
)

# Wait until the client is fully connected to the drone
while not drone_client.is_fully_connected:
    time.sleep(0.1)

# Create and start the client that will connect to the motion capture system
if use_mocap:
    mocap_client = QualisysClient([{'name': marker_deck_name, 'callback': None}])

# Pause before takeoff
drone_client.stop(1.0)

#
# FIXME: Insert move commands here to fly...
#
#   drone_client.move(0.0, 0.0, 0.3, 0.0, 1.0)
#

# Pause after landing
drone_client.stop(1.0)

# Disconnect from the drone
drone_client.disconnect()

# Disconnect from the motion capture system
if use_mocap:
    mocap_client.close()

# Assemble flight data from both clients
data = {}
data['drone'] = drone_client.data
data['mocap'] = mocap_client.data.get(marker_deck_name, {}) if use_mocap else {}
data['bodies'] = mocap_client.data if use_mocap else {}

# Write flight data to a file
with open('hardware_data.json', 'w') as outfile:
    json.dump(data, outfile, sort_keys=False)