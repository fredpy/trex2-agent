## Presentation ##

This project is taking its roots from the same source as [trex-autonomy](http://code.google.com/p/trex-autonomy/) project created by Willow Garage.

The reason for creating this new project is twofold :
<li> trex-autonomy has been dead for months now since his main contributor has moved on to other professional projects </li>
<li> this new version is much more aligned with recent publications on TREX architecture (<a href='References#aamas10.md'>aamas 2010</a>) and  provide a good basis for future extensions </li>

This project is also created and maintained by [MBARI](http://www.mbari.org/) engineers --  where TREX was initially designed -- with a strong desire to disseminate this agent architecture for the community.

Finally while former versions of TREX were tied up to the [europa-pso](http://code.google.com/p/europa-pso/) planner, this new version is independent of the planner that being used for the deliberation inside the reactors that are the basic building blocks of an agent. even though we are planning to provide a connection to europa this will remain an extra plug-in that is not necessary to run the agent and exist mostly for allowing people to use planning inside our agent without needing to integrate a planner on their own; and also for illustration purpose for whoever wants to integrate another automated planner.


---

### Latest version ###

The new version [0.5.4](https://trex2-agent.googlecode.com/svn/package_files/trex-0.5.4.tar.gz) is now available.

Past versions archives (since 0.5.2 as it is when google code stopped the Download service) can be found on the [svn repository](https://code.google.com/p/trex2-agent/source/browse/#svn%2Fpackage_files) or by getting their corresponding [tag](https://code.google.com/p/trex2-agent/source/browse/#svn%2Ftags) (all official version are tagged `VERSION_*`).

#### [0.5.4](https://trex2-agent.googlecode.com/svn/package_files/trex-0.5.4.tar.gz) ####

Patches for boost 1.56.0 support.

#### [0.5.3](https://trex2-agent.googlecode.com/svn/package_files/trex-0.5.3.tar.gz) ####

Many patches related to the lightswitch example (sample.cfg) and issues related to issues discovered while trying to use this mission:
  * corrected a bug where the tree clock could be updated erroneously before the mission beginning
  * Ensured that lightswitch model on europa 2.6 respect the chaining of effect/condition from a goal to command
    * in order to dispatch an external state request as early as possible this needs to be connected through
> > > a sequence of actions to a goal this was not the case
  * Removed pseudo tests for tree from this mission so it is more clean to follow and do not produce pseudo warnings
  * Limited test and test2 europa reactors from sample.cfg to deduce synchronized state for a limited number of   steps in order to avoid them to block as they are getting lost in searching for a solution.

#### 0.5.2 ####

Version used during Sunfish lsts experiment most updates relate the LSTS third party plug-ins but also include
patches on the core and europa components.

Mostly minor improvements patches:
  * Improvement of europa reactor past archival policy for better performance during long missions
  * Added the ability to change a reactor lookahead and latency during runtime (experimental)
  * Several tweaks and improvement mostly in  the lsts related plug-ins


#### 0.5.1 ####
Many alterations on the way trex core work with specifically:
  * some operations(communication between reactors and logging) being deported in separate threads
  * added a plugin based on Wt and providing a basic REST API
  * initial ROS interfacing prototype
  * several improvements on the europa reactor staking advantage of the newly introduced notion of action in europa 2.6
  * special third\_party plugins for connection to existing systems
  * support for implementing and compiling a plug-in outside of trex core source tree
  * and many more patches/improvements

versions 0.4.x were only present on svn and were mostly patches and transitional code toward this version

#### 0.3.2 ####
Many patches in the europa reactor. Most of them are related to avoid europa crashing trex after database relaxation.


#### 0.3.1 ####
Two critical patches correcting issues in if 0.3.0 was not compiled in Debug mode
  * added include `<memory>` before using a std::auto\_ptr in Assembly.hh
  * Made sure that all the europa source include `<trex/europa/config.hh>` before including any PLASMA header

#### 0.3.0 ####
The main focus of this version was to :
  * improve error handling on the agent (both cycle detection and exception thrown by a reactor)
  * re-implement the europa reactor
    1. now resolving synchronization is handled through a europa solver
    1. it also support the new europa 2.6
  * new additions on the witre reactor to visualize the agent execution on a web page

To know how to install it you can :
  * read the INSTALL file on the archive
  * or check the InstallGuide

---


### support ###

User mailing list : You can send an email to [trex2-users](mailto:trex2-users@googlegroups.com) mailing list or  [subscribe](http://groups.google.com/group/trex2-users) to it.

### Documentation ###

You can look at the doxygen generated documentation [here](http://trex2-agent.googlecode.com/svn/doc/html/index.html). This documentation is usually generated from the head. But as it can take a while to generate, we update it only when major changes appears.