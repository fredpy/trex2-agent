#! /usr/bin/python

import trex

class test_reactor(trex.transaction.reactor):
    "A simple reactor that does nothing"
    def __init__(self,cfg):
        "constructor for trex reactor factory"
        # always explicitley call the base constructor
        trex.transaction.reactor.__init__(self,cfg)
        # declare test_<name> as Internal
        self.declare('test_{}'.format(self.name))
    def synchronize(self):
        "synchronization callback should always be redefined"
        # build an observation on test_<name>
        o = trex.transaction.obs('test_{}'.format(self.name), 'Tick')
        # post the observation and display it in TREX.log
        self.post(o, True)
        # return true to confirma that we did not fail
        return True


# Load the agent based on sample.cfg
agent = trex.agent.agent('sample')
# sample of xml tag for a python reactor based on test_reactor class
cfg = trex.utils.xml.from_str('<PyReactor name="foo" python_class="test_reactor" lookahead="0" latency="0"/>')
# create a clock updating avery 500ms (2Hz)
clk = trex.agent.rt_clock(500)

# Add the reactor defined in cfg to the agent (this should create a new test_reactor)
agent.add_reactor(iter(cfg).next())
# Set the clock of the agent to clk
agent.set_clock(clk)
# run the agent mission
agent.run()
