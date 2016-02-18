#!/bin/bash
rosrun snippets make_structs.py --gen-cpp --definition `rospack find usbl_comms`/definitions/caddy/$1.xml --output-dir `rospack find usbl_comms`/include/labust/comms/caddy/detail/ --output-name caddy_messages
mkdir -p `rospack find usbl_comms`/java
rosrun snippets make_structs.py --gen-java --definition `rospack find usbl_comms`/definitions/caddy/$1.xml --output-dir `rospack find usbl_comms`/java/ --java-package hr.fer.labust.comms.caddy

