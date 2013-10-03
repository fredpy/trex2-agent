#!/bin/bash

PORT=8888
HOST=127.0.0.1

curl -X POST -d '
{"on": "spotSim", "pred": "position", 
"Variable": [
  {"name": "latitude", "float": { "value": "0.718799803" }},
  {"name": "longitude", "float": { "value": "-0.151943914" }}
]}

' http://$HOST:$PORT/rest/goal --header 'Content-Type:application/json'
