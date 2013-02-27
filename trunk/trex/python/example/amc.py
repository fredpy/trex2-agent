#! /usr/bin/python

import trex

def run_agent(file_name):
    agent = trex.agent.agent(file_name)
    default_clock = trex.agent.rt_clock(1000)
    agent.set_clock(default_clock)
    agent.run()
    

