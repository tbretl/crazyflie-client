###################################
# IMPORTS

import json
import os
from ae483.clients import QualisysClient
from ae483.myclients import MyCrazyflieClient


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
use_mocap = False

# Specify whether or not to use a custom controller
use_controller = False

# Specify whether or not to use a custom observer
use_observer = False

# Specify the name of the file in which to save flight data
data_filename = 'hardware_data.json'

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
drone_client = MyCrazyflieClient(
    uri,
    use_controller=use_controller,
    use_observer=use_observer,
    marker_deck_ids=marker_deck_ids if use_mocap else None,
    variables=variables,
)

# Create this now so that it always exists, even if we never get far enough to
# connect to the motion capture system
mocap_client = None

# Everything from here until "finally" is what happens during your flight. If
# anything goes wrong — including if you press Ctrl-C — the code in the
# "finally" block still runs, stopping the motors, disarming the drone, and
# disconnecting.
try:
    # Wait until the client is fully connected to the drone and until the state
    # estimate has had time to converge
    drone_client.wait_until_ready()

    # Create and start the client that will connect to the motion capture system
    if use_mocap:
        mocap_client = QualisysClient([{'name': marker_deck_name, 'callback': None}])

    # Arm the drone. Brushless drones will not spin their motors until they are
    # armed. Brushed drones do not need to be armed, but arming them does no
    # harm, so the same flight code works for both.
    drone_client.arm()

    # Pause before takeoff
    drone_client.stop(1.0)

    #
    # FIXME: Insert move commands here to fly...
    #
    #   drone_client.move(0.0, 0.0, 0.3, 0.0, 1.0)
    #

    # Pause after landing
    drone_client.stop(1.0)

except KeyboardInterrupt:
    print('\nInterrupted — stopping the motors and saving whatever data were collected.')

finally:
    # Stop the motors, disarm, and disconnect from the drone. Each step is
    # guarded so that a failure in one of them cannot prevent the others — and,
    # in particular, cannot prevent your flight data from being saved.
    try:
        drone_client.close()
    except Exception as e:
        print(f'Error while closing the connection to the drone: {e}')

    # Disconnect from the motion capture system
    if mocap_client is not None:
        try:
            mocap_client.close()
        except Exception as e:
            print(f'Error while closing the connection to the motion capture system: {e}')

    # Assemble flight data from both clients
    data = {}
    data['drone'] = drone_client.data
    data['mocap'] = mocap_client.data.get(marker_deck_name, {}) if use_mocap and mocap_client is not None else {}
    data['bodies'] = mocap_client.data if use_mocap and mocap_client is not None else {}

    # Write flight data to a file. We write to a temporary file first and then
    # rename it, which is an operation the operating system does all at once.
    # That way, if anything interrupts the writing, you are left with your
    # previous data file rather than with a half-written one that cannot be
    # read at all.
    temporary_filename = data_filename + '.partial'
    with open(temporary_filename, 'w') as outfile:
        json.dump(data, outfile, sort_keys=False)
    os.replace(temporary_filename, data_filename)
    print(f'Wrote flight data to {data_filename}')
