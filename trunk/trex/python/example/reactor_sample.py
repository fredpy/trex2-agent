#! /usr/bin/python

import trex

class test_reactor(trex.transaction.reactor):
    "A simple reactor that does nothing"
    def __init__(self,cfg):
        "constructor for trex reactor factory"
        trex.transaction.reactor.__init__(self,cfg)
        self.declare('test_{}'.format(self.name))
    def synchronize(self):
        "synchronization callback should always be redefined"
        o = trex.transaction.obs('test_{}'.format(self.name), 'Tick')
        self.post(o, True)
        return True


agent = trex.agent.agent('sample')
cfg = trex.utils.xml.from_str('<PyReactor name="foo" python_class="test_reactor" lookahead="0" latency="0"/>')
clk = trex.agent.rt_clock(500) # run at 2Hz

agent.add_reactor(iter(cfg).next())
agent.set_clock(clk)
agent.run()
