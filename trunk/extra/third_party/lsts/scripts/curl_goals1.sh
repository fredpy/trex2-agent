#!/bin/bash

PORT=8888
HOST=127.0.0.1

curl -X POST -d '
{"on": "navigator", "pred": "At", 
"Variable": [
  {"name": "latitude", "float": { "value": "0.718799803" }},
  {"name": "longitude", "float": { "value": "-0.151943914" }},
  {"name": "z", "float": { "value": "5" }},
  {"name": "speed", "float": { "value": "1.5" }},
  {"name": "duration", "int": {"max": 200}} 
]}

' http://$HOST:$PORT/rest/goal --header 'Content-Type:application/json'
