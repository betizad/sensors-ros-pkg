#!/bin/bash
rosrun snippets make_structs.py --definition `rospack find usbl_comms`/definitions/caddy/Messages.xml --output-dir `rospack find usbl_comms`/include/labust/comms/caddy/detail/ --output-name caddy_messages

