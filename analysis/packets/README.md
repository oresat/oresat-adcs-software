

# init_raw

This is for saving the raw json from satnogs. We save the json string to simplify comparison.

Files generated:
- `init_raw.json`: progressive save of api results
- `init_raw_finished.json`: all saved api results




# prepend_raw

We assume that all telemetry from satnogs api is already chronological. 
Thus, we only need to check if any particular packet is the same as the newest packet from the saved packets.

Files needed:
- `ref_telemetry.json`: this is a rename of `init_raw_finished.json`

Files generated:
- `prepended_raw_finished.json`: Output of saved api results



# decode

Takes raw json frames and decodes them. Requires oresat-configs




