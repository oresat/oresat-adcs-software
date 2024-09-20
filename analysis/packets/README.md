# init_raw

This is for saving the raw json from satnogs. We save the json string to simplify comparison.


# prepend_raw

We assume that all telemetry from satnogs api is already chronological. 
Thus, we only need to check if any particular packet is the same as the newest packet from the saved packets.

# decode

Takes raw json frames and decodes them. Requires oresat-configs


