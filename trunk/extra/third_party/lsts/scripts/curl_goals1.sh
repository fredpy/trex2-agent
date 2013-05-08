#!/bin/bash

PORT=8888
HOST=127.0.0.1

curl -X POST -d '{"on": "navigator", "pred": "At", "Variable": [{"name": "speed", "float": { "value": "1.5" }}, {"name": "duration", "int": {"max": 200}} ]}' http://$HOST:$PORT/rest/goal --header 'Content-Type:application/json'

