#! /usr/bin/python

import trex

class log_printer(trex.utils.log_handler):
    "A simple trex log handler that print log messages"
    def new_entry(self, e):
        date=''
        if( e.is_dated ):
            date = '['+ str(e.date()) + ']'
        print '{}[{}]{}: {}'.format(date, e.source(), e.kind(), e.content())

trex_log = log_printer();

def run_agent(file_name):
    "batch execution of an agent"
    "This call executes an agent based on the config sepcified in the file file_name"
    agent = trex.agent.agent(file_name)
    default_clock = trex.agent.rt_clock(1000)
    agent.set_clock(default_clock)
    agent.run()
    

