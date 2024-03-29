/** @file TeleoReactor.cc
 * @brief Provides implementation for TeleoReactor
 *
 * @author Conor McGann @& Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <cmath>
#include <utility>

// #include <boost/chrono/clock_string.hpp>

#include "TeleoReactor.hh"
#include <trex/domain/FloatDomain.hh>

#include <bitset>

using TREX::utils::Symbol;
namespace utils = TREX::utils;

namespace TREX {
namespace transaction {

class TeleoReactor::Logger {
public:
  Logger(std::string const &dest, boost::asio::io_service &io);
  ~Logger();

  void provide(Symbol const &name, bool goals, bool plan);
  void unprovide(Symbol const &name);

  void use(Symbol const &name, bool goals, bool plan);
  void unuse(Symbol const &name);

  void init(TICK val);
  void newTick(TICK val);
  void synchronize();

  void failed();
  void has_work();
  void work(bool ret);
  void step();

  void observation(Observation const &obs);
  void request(goal_id const &goal);
  void recall(goal_id const &goal);

  void comment(std::string const &msg);
  void notifyPlan(goal_id const &tok);
  void cancelPlan(goal_id const &tok);

  void latency_updated(TICK val);
  void horizon_updated(TICK val);

private:
  boost::asio::io_service::strand m_strand;
  utils::async_ofstream m_file;

  enum { header = 0, tick = 1, tick_opened = 2, in_phase = 3, has_data = 4 };

  std::bitset<5> m_flags;

  enum tick_phase { in_init, in_new_tick, in_synchronize, in_work, in_step };

  tick_phase m_phase;
  TICK m_current;

  void goal_event(std::string tag, goal_id g, bool full);

  template <class Handler,
            typename = std::enable_if_t<std::is_invocable_v<Handler>>>
  void post_event(Handler &&fn) {
    boost::asio::post(m_strand, std::forward<Handler>(fn));
  }
  template <class Handler,
            typename = std::enable_if_t<std::is_invocable_v<Handler>>>
  void phase_event(Handler &&fn) {
    open_phase();
    std::invoke(std::forward<Handler>(fn));
  }

  void open_tick();
  void open_phase();
  void close_phase();
  void close_tick();

  void set_tick(TICK val, tick_phase p);
  void set_phase(tick_phase p);
  void direct_write(std::string const &content, bool nl);
};

} // namespace transaction
} // namespace TREX

using namespace TREX::transaction;

/*
 * class TREX::transaction::ReactorException
 */
ReactorException::ReactorException(TeleoReactor const &r,
                                   std::string const &msg) throw()
    : GraphException(r.m_graph, r.getName().str(), msg) {}

/*
 * class TREX::transaction::DispatchError
 */
// statics
std::string DispatchError::build_msg(goal_id const &g,
                                     std::string const &msg) throw() {
  std::ostringstream oss;
  oss << "While dispatching ";
  if (!!g)
    oss << g->object() << '.' << g->predicate();
  oss << '[' << g << "]: " << msg;
  return oss.str();
}

/*
 * class TREX::transaction::details::external
 */

bool TREX::transaction::details::external::cmp_goals(IntegerDomain const &a,
                                                     IntegerDomain const &b) {
  // sorting order
  //   - based on upperBound
  //   - if same upperBound : sorted based on lower bound
  //
  // this way I can safely update lower bounds without impacting tokens order
  return a.upperBound() < b.upperBound() ||
         (a.upperBound() == b.upperBound() && a.lowerBound() < b.lowerBound());
}

// structors

TREX::transaction::details::external::external() {
  m_pos = m_last; // may not be necessary but I just want  ot be sure ...
}

TREX::transaction::details::external::external(details::external const &other)
    : m_pos(other.m_pos), m_last(other.m_last) {}

TREX::transaction::details::external::external(
    details::external_set::iterator const &pos,
    details::external_set::iterator const &last)
    : m_pos(pos), m_last(last) {}

// manipulators

details::goal_queue::iterator
details::external::lower_bound(IntegerDomain const &dom) {
  details::goal_queue::iterator i = m_pos->second.begin();

  for (; m_pos->second.end() != i && cmp_goals(i->first->getStart(), dom); ++i)
    ;
  return i;
}

// modifiers

bool details::external::post_goal(goal_id const &g) {
  IntegerDomain const &g_start(g->getStart());
  // locate goal position
  details::goal_queue::iterator i = lower_bound(g_start);

  // check that it is not already posted
  for (; m_pos->second.end() != i && !cmp_goals(g_start, i->first->getStart());
       ++i)
    if (g == i->first)
      return false;
  // insert the new goal
  m_pos->second.insert(i, std::make_pair(g, true));
  if (m_pos->first.client().is_verbose()) {
    syslog(info) << m_pos->first.client().getName() << " added "
                 << g->predicate() << '[' << g << "] to the pending queue of "
                 << m_pos->first.name();
  }
  return true;
}

void details::external::dispatch(TICK current, details::goal_queue &sent) {
  details::goal_queue::iterator i = m_pos->second.begin();
  IntegerDomain dispatch_w = m_pos->first.dispatch_window(current);

  for (; m_pos->second.end() != i &&
         i->first->startsBefore(dispatch_w.upperBound());) {
    bool future = i->first->startsAfter(current);

    if (future || i->first->endsAfter(current + 1)) {
      // Need to check for dispatching
      if (i->second && m_pos->first.accept_goals()) {
        if (m_pos->first.client().is_verbose())
          syslog(info) << "Dispatching " << i->first->predicate() << '['
                       << (i->first) << "] on \"" << m_pos->first.name()
                       << "\".";
        bool posted = false;
        try {
          m_pos->first.request(i->first);
          posted = true;
          i = m_pos->second.erase(i);
        } catch (utils::Exception const &e) {
          syslog(warn) << "Exception received while sending request: " << e;
        } catch (std::exception const &se) {
          syslog(warn) << "C++ exception received while sending request: "
                       << se.what();
        } catch (...) {
          syslog(warn) << "Unknown exception received while sending request.";
        }
        if (!posted) {
          syslog(warn) << "Marking goal as non-postable.";
          i->second = false;
        }
      } else
        ++i;
    } else if (!future) {
      syslog(warn) << "Goal " << i->first->predicate() << '[' << (i->first)
                   << "] is in the past: removing it\n\t" << (*(i->first));
      i = m_pos->second.erase(i);
    } else if (!m_pos->first.accept_goals())
      break; // no need to  look further ... this guy do not accept goals
  }
}

void details::external::unblock() {
  // just mark all the
  for (details::goal_queue::iterator i = m_pos->second.begin();
       m_pos->second.end() != i; ++i)
    i->second = true;
}

void details::external::recall(goal_id const &g) {
  IntegerDomain const &g_start(g->getStart());
  // locate goal position
  details::goal_queue::iterator i = lower_bound(g_start);
  for (; m_pos->second.end() != i && !cmp_goals(g_start, i->first->getStart());
       ++i) {
    if (g == i->first) {
      // was still pending => just remove it
      m_pos->second.erase(i);
      return;
    }
  }
  // not found => send a recall
  m_pos->first.recall(g);
}

void details::external::increment() {
  if (valid()) {
    ++m_pos;
  }
}

// observers

bool details::external::equal(details::external const &other) const {
  if (valid())
    return m_pos == other.m_pos;
  else
    return !other.valid();
}

TREX::utils::log::stream details::external::syslog(utils::Symbol const &kind) {
  return m_pos->first.client().syslog(m_pos->first.name(), kind);
}

Relation const &details::external::dereference() const {
  return *(m_pos->first);
}

/*
 * class TREX::transaction::TeleoReactor
 */

utils::Symbol const TeleoReactor::obs("ASSERT");
utils::Symbol const TeleoReactor::plan("PLAN");

// structors

TeleoReactor::TeleoReactor(TeleoReactor::xml_arg_type &arg, bool loadTL,
                           bool log_default)
    : m_inited(false), m_firstTick(true), m_graph(*(arg.second)),
      m_have_goals(0),
      m_verbose(utils::parse_attr<bool>(arg.second->is_verbose(),
                                        xml_factory::node(arg), "verbose")),
      m_trLog(NULL),
      m_name(utils::parse_attr<Symbol>(xml_factory::node(arg), "name")),
      m_latency(utils::parse_attr<TICK>(xml_factory::node(arg), "latency")),
      m_maxDelay(0),
      m_lookahead(utils::parse_attr<TICK>(xml_factory::node(arg), "lookahead")),
      m_nSteps(0), m_past_deadline(false), m_validSteps(0),
      m_stat_log(m_log->context()) {
  boost::property_tree::ptree::value_type &node(xml_factory::node(arg));

  utils::LogManager::path_type fname = file_name("stat.csv");
  m_stat_log.open(fname.c_str());
  m_stat_log << "tick, tick_ns, tick_rt_ns, synch_ns, synch_rt_ns, delib_ns, "
                "delib_rt_ns, n_steps\n";

  if (utils::parse_attr<bool>(log_default, node, "log")) {
    std::string base = getName().str() + ".tr.log";
    fname = manager().file_name(base);
    m_trLog = new Logger(fname.string(), manager().context());
    utils::LogManager::path_type cfg = manager().file_name("cfg"),
                                 pwd = std::filesystem::current_path(),
                                 short_name(base), location("../" + base);
    std::filesystem::current_path(cfg);
    try {
      create_symlink(location, short_name);
    } catch (...) {
    }
    std::filesystem::current_path(pwd);
    syslog(info) << "Transactions logged to " << fname;
  }

  if (loadTL) {
    utils::Symbol tl_name;
    // Add external file content
    utils::ext_xml(node.second, "config");

    for (boost::property_tree::ptree::iterator i = node.second.begin();
         node.second.end() != i; ++i) {
      if (utils::is_tag(*i, "External")) {
        tl_name = utils::parse_attr<Symbol>(*i, "name");
        if (tl_name.empty())
          throw utils::XmlError(*i, "Timelines cannot have an empty name");
        use(tl_name, utils::parse_attr<bool>(true, *i, "goals"),
            utils::parse_attr<bool>(false, *i, "listen"));
      } else if (utils::is_tag(*i, "Internal")) {
        tl_name = utils::parse_attr<Symbol>(*i, "name");
        if (tl_name.empty())
          throw utils::XmlError(*i, "Timelines cannot have an empty name");
        provide(tl_name);
      }
    }
  }
}

TeleoReactor::TeleoReactor(graph *owner, Symbol const &name, TICK latency,
                           TICK lookahead, bool log)
    : m_inited(false), m_firstTick(true), m_graph(*owner), m_have_goals(0),
      m_verbose(owner->is_verbose()), m_trLog(NULL), m_name(name),
      m_latency(latency), m_maxDelay(0), m_lookahead(lookahead), m_nSteps(0),
      m_stat_log(m_log->context()) {
  utils::LogManager::path_type fname = file_name("stat.csv");
  m_stat_log.open(fname.string());

  if (log) {
    fname = manager().file_name(getName().str() + ".tr.log");
    m_trLog = new Logger(fname.string(), manager().context());
    syslog(info) << "Transactions logged to " << fname;
  }
}

TeleoReactor::~TeleoReactor() {
  isolate(false);
  if (!m_firstTick) {
    m_stat_log << ", " << m_deliberation_usage.count() << ", "
               << m_delib_rt.count() << ", " << m_tick_steps << std::endl;
    m_stat_log.close();
  }

  if (NULL != m_trLog) {
    delete m_trLog;
  }
}

// observers

TeleoReactor::size_type TeleoReactor::count_internal_relations() const {
  size_type result(0);

  for (internal_set::const_iterator i = m_internals.begin();
       m_internals.end() != i; ++i)
    result += (*i)->size();
  return result;
}

bool TeleoReactor::internal_sync(TREX::utils::Symbol name) const {
  return m_internals.end() != m_internals.find(name);
}

bool TeleoReactor::external_sync(TREX::utils::Symbol name) const {
  return m_externals.end() != m_externals.find(name);
}

bool TeleoReactor::isInternal(TREX::utils::Symbol const &timeline) const {
  return utils::strand_run(m_graph.strand(), [this, timeline]() -> bool {
    return internal_sync(timeline);
  });
}

bool TeleoReactor::isExternal(TREX::utils::Symbol const &timeline) const {
  return utils::strand_run(
      m_graph.strand(), [this, timeline] { return external_sync(timeline); });
}

details::external TeleoReactor::ext_begin() {
  return details::external(m_externals.begin(), m_externals.end());
}

details::external TeleoReactor::ext_end() {
  return details::external(m_externals.end(), m_externals.end());
}

details::external TeleoReactor::find_external(TREX::utils::Symbol const &name) {
  return details::external(m_externals.find(name), m_externals.end());
}

// modifers/callbacks

void TeleoReactor::reset_deadline() {
  // initialize the deliberation parameters
  m_deadline = getCurrentTick() + 1 + getLatency();
  m_nSteps = 0;
  m_past_deadline = false;
}

bool TeleoReactor::have_goals() {
  utils::SharedVar<size_t>::scoped_lock lock(m_have_goals);
  return 0 < *m_have_goals;
}

void TeleoReactor::goal_flush(std::list<goal_id> &a, std::list<goal_id> &dest) {
  {
    utils::SharedVar<size_t>::scoped_lock lock(m_have_goals);
    *m_have_goals -= a.size();
  }
  std::swap(a, dest);
}

void TeleoReactor::queue(std::list<goal_id> &l, goal_id g) {
  {
    utils::SharedVar<size_t>::scoped_lock lock(m_have_goals);
    *m_have_goals += 1;
  }
  l.push_back(g);
}

void TeleoReactor::queue_goal(goal_id g) {
  boost::asio::dispatch(m_graph.strand(),
                        [this, g] { queue(m_sync_goals, g); });
}

void TeleoReactor::queue_recall(goal_id g) {
  boost::asio::dispatch(m_graph.strand(),
                        [this, g] { queue(m_sync_recalls, g); });
}

void TeleoReactor::queue_token(goal_id g) {
  boost::asio::dispatch(m_graph.strand(), [this, g] { queue(m_sync_toks, g); });
}

void TeleoReactor::queue_cancel(goal_id g) {
  boost::asio::dispatch(m_graph.strand(),
                        [this, g] { queue(m_sync_cancels, g); });
}

double TeleoReactor::workRatio() {
  std::list<goal_id> tmp;

  if (have_goals()) {
    // Start to flush goals
    utils::strand_run(m_graph.strand(),
                      [this, &tmp] { goal_flush(m_sync_goals, tmp); });

    while (!tmp.empty()) {
      handleRequest(tmp.front());
      tmp.pop_front();
    }
  }
  if (have_goals()) {
    utils::strand_run(m_graph.strand(),
                      [this, &tmp] { goal_flush(m_sync_recalls, tmp); });
    while (!tmp.empty()) {
      handleRecall(tmp.front());
      tmp.pop_front();
    }
  }
  if (have_goals()) {
    // Start to flush plan tokens
    utils::strand_run(m_graph.strand(),
                      [this, &tmp] { goal_flush(m_sync_toks, tmp); });
    while (!tmp.empty()) {
      newPlanToken(tmp.front());
      tmp.pop_front();
    }
  }
  if (have_goals()) {
    // Start to flush plan tokens
    utils::strand_run(m_graph.strand(),
                      [this, &tmp] { goal_flush(m_sync_cancels, tmp); });
    while (!tmp.empty()) {
      cancelPlanToken(tmp.front());
      tmp.pop_front();
    }
  }

  bool ret = false;

  if (NULL != m_trLog)
    m_trLog->has_work();
  try {
    ret = hasWork();
    if (NULL != m_trLog)
      m_trLog->work(ret);

    if (ret) {
      double ret = m_deadline;
      ret -= getCurrentTick();
      if (ret < 0.0 && m_nSteps > 0) {
        if (!m_past_deadline) {
          m_past_deadline = true;
          m_validSteps = m_nSteps;
          syslog(warn) << " Reactor is now exceeding its deliberation latency ("
                       << getLatency()
                       << ")\n\tNumber of steps within its latency: "
                       << m_validSteps;
        }
        ret = m_nSteps + 1;
      } else {
        ret += 1.0;
        ret *= m_nSteps + 1;
      }
      return 1.0 / ret;
    } else {
      // Dispatched goals management
      details::external i = ext_begin();
      details::goal_queue dispatched; // store the goals that got dispatched
                                      // on this tick ...
                                      // I do nothing with it for now

      // Manage goal dispatching
      for (; i.valid(); ++i)
        i.dispatch(getCurrentTick() + 1, dispatched);
    }
  } catch (std::exception const &se) {
    syslog(warn) << "Exception during hasWork question: " << se.what();
  } catch (...) {
    syslog(warn) << "Unknown Exception during hasWork question";
  }
  if (m_past_deadline) {
    syslog(warn) << "Reactor needed to deliberate " << (m_nSteps - m_validSteps)
                 << " extra steps spread other "
                 << (getCurrentTick() - m_deadline)
                 << " ticks after its latency.";
  }
  reset_deadline();
  return NAN;
}

void TeleoReactor::postObservation(Observation const &obs, bool verbose) {
  utils::strand_run(m_graph.strand(), [this, &obs, verbose] {
    auto i = m_internals.find(obs.object());
    if (m_internals.end() == i) {
      throw SynchronizationError(*this, "attempted to post observation on " +
                                            obs.object().str() +
                                            " which is not Internal.");
    }
    (*i)->postObservation(obs, verbose);
    m_updates.insert(*i);
  });
}

bool TeleoReactor::postGoal(goal_id const &g) {
  if (!g)
    throw DispatchError(*this, g, "Invalid goal Id");
  return utils::strand_run(m_graph.strand(), [this, g]() -> bool {
    details::external tl{m_externals.find(g->object()), m_externals.end()};
    if (!tl.valid()) {
      throw DispatchError(*this, g,
                          "Goals can only be posted on External timelines");
    }
    if (nullptr != m_trLog) {
      m_trLog->request(g);
    }
    return tl.post_goal(g);
  });
}

goal_id TeleoReactor::postGoal(Goal const &g) {
  goal_id tmp(new Goal(g));

  if (postGoal(tmp))
    return tmp;
  else {
    // should never happen !?
    return goal_id();
  }
}

goal_id
TeleoReactor::parse_goal(boost::property_tree::ptree::value_type const &g) {
  return getGraph().parse_goal(g);
}

bool TeleoReactor::postRecall(goal_id const &g) {
  if (!g)
    return false;
  return utils::strand_run(m_graph.strand(), [this, g]() -> bool {
    if (details::external tl{m_externals.find(g->object()), m_externals.end()};
        tl.valid()) {
      if (nullptr != m_trLog) {
        m_trLog->recall(g);
      }
      tl.recall(g);
      return true;
    }
    return false;
  });
}

bool TeleoReactor::postPlanToken(goal_id const &t) {
  if (!t)
    throw DispatchError(*this, t, "Invalid token id");
  return utils::strand_run(m_graph.strand(), [this, t]() -> bool {
    // Look for the internal timeline
    internal_set::const_iterator tl = m_internals.find(t->object());
    if (m_internals.end() == tl)
      throw DispatchError(
          *this, t, "plan tokens can only be posted on Internal timelines.");
    else if (t->getEnd().upperBound() > getCurrentTick()) {
      if (nullptr != m_trLog)
        m_trLog->notifyPlan(t);
      return (*tl)->notifyPlan(t);
    }
    return false;
  });
}

goal_id TeleoReactor::postPlanToken(Goal const &g) {
  goal_id tmp(new Goal(g));

  if (postPlanToken(tmp))
    return tmp;
  else
    return goal_id();
}

void TeleoReactor::cancelPlanToken(goal_id const &g) {
  if (g) {
    utils::strand_run(m_graph.strand(), [this, g] {
      if (internal_set::const_iterator tl = m_internals.find(g->object());
          m_internals.end() != tl) {
        // do something
        if (nullptr != m_trLog)
          m_trLog->cancelPlan(g);

        (*tl)->cancelPlan(g);
      }
    });
  }
}

TICK TeleoReactor::getFinalTick() const {
  TICK g_final = m_graph.finalTick();

  if (!m_finalTick || g_final < *m_finalTick)
    return g_final;
  else
    return *m_finalTick;
}

void TeleoReactor::setMaxTick(TICK max) {
  if (!m_finalTick || max < *m_finalTick) {
    syslog(warn) << "Restricted reactor final tick to " << max;
    m_finalTick = max;
  }
}

TICK TeleoReactor::update_latency(TICK new_latency) {
  // NOTE: this is not thread_safe
  std::swap(m_latency, new_latency);
  if (new_latency != m_latency) {
    syslog(info) << "latency updated from " << new_latency << " to "
                 << m_latency;
    for (internal_set::iterator i = m_internals.begin(); m_internals.end() != i;
         ++i)
      (*i)->latency_update(new_latency + m_maxDelay);
    if (m_trLog)
      m_trLog->latency_updated(m_latency);
  }
  return new_latency;
}

TICK TeleoReactor::update_horizon(TICK new_horizon) {
  // NOTE: this is not thread safe
  std::swap(m_lookahead, new_horizon);
  if (m_lookahead != new_horizon) {
    syslog(info) << "horizon updated from " << new_horizon << " to "
                 << m_lookahead;
    if (m_trLog)
      m_trLog->horizon_updated(m_lookahead);
  }
  return new_horizon;
}

bool TeleoReactor::initialize(TICK final) {
  if (m_inited) {
    syslog(error) << "Attempted to initalize this reactor twice.";
    return false;
  }
  m_initialTick = m_obsTick = getCurrentTick();

  if (!m_finalTick || final < *m_finalTick)
    m_finalTick = final;
  syslog(info) << "Creation tick is " << getInitialTick();
  syslog(info) << "Execution latency is " << getExecLatency();
  // syslog()<<"Clock used for stats is
  // "<<boost::chrono::clock_string<stat_clock, char>::name();
  try {
    if (NULL != m_trLog)
      m_trLog->init(m_initialTick);
    handleInit(); // allow derived class initialization
    m_firstTick = true;
    m_inited = true;
    return true;
  } catch (TREX::utils::Exception const &e) {
    syslog(error) << "Exception caught during init :\n" << e;
  } catch (std::exception const &se) {
    syslog(error) << "C++ exception caught during init :\n" << se.what();
  } catch (...) {
    syslog(error) << "Unknown exception caught during init";
  }
  return false;
}

bool TeleoReactor::newTick() {
  if (m_firstTick) {
    m_obsTick = getCurrentTick();
    if (m_obsTick != m_initialTick) {
      syslog(warn) << "Updating initial tick from " << m_initialTick << " to "
                   << getCurrentTick();
      m_initialTick = m_obsTick;
    }
    reset_deadline();
    TICK final = getFinalTick();

    if (final < m_graph.finalTick())
      syslog(warn) << "Reactor final tick is before agent's one:\n\t"
                   << date_str(final) << " (" << final << ").";

    m_firstTick = false;
  } else
    m_stat_log << ", " << m_deliberation_usage.count() << ", "
               << m_delib_rt.count() << ", " << m_tick_steps << std::endl;
  m_tick_steps = 0;

  if (getCurrentTick() > getFinalTick()) {
    syslog(warn) << "This reactor reached its final tick.";
    return false;
  }

  //  if( m_deliberation_usage > stat_duration::zero() )
  //    syslog("stats")<<"
  //    delib="<<boost::chrono::duration_short<<m_deliberation_usage;
  m_deliberation_usage = stat_duration::zero();
  m_delib_rt = rt_clock::duration::zero();

  if (NULL != m_trLog)
    m_trLog->newTick(getCurrentTick());

  try {
    {
      utils::chronograph<stat_clock> stat_chron(m_start_usage);
      utils::chronograph<rt_clock> rt_chron(m_start_rt);

      handleTickStart(); // allow derived class processing
    }

    // Dispatched goals management
    details::external i = ext_begin();
    details::goal_queue
        dispatched; // store the goals that got dispatched on this tick ...
                    // I do nothing with it for now

    // Manage goal dispatching
    for (; i.valid(); ++i)
      i.dispatch(getCurrentTick(), dispatched);
    return true;
  } catch (TREX::utils::Exception const &e) {
    syslog(error) << "Exception caught during new tick:\n" << e;
  } catch (std::exception const &se) {
    syslog(error) << "C++ exception caught during new tick:\n" << se.what();
  } catch (...) {
    syslog(error) << "Unknown exception caught during new tick";
  }
  return false;
}

void TeleoReactor::doNotify() {
  std::list<Observation> obs;
  utils::strand_run(m_graph.strand(), [this, &obs] {
    for (auto &ext : m_externals) {
      if (ext.first.lastObsDate() == getCurrentTick()) {
        obs.push_back(ext.first.lastObservation());
      }
    }
  });
  for (auto &o : obs) {
    notify(o);
  }
}

bool TeleoReactor::doSynchronize() {
  if (NULL != m_trLog)
    m_trLog->synchronize();
  bool stat_logged = false;

  try {
    TICK now = getCurrentTick();

    bool success;
    {
      // collect information from external timelines
      doNotify();
      {
        // measure timing only for synchronization call
        utils::chronograph<rt_clock> real_time(m_synch_rt);
        utils::chronograph<stat_clock> usage(m_synch_usage);
        success = synchronize();
      }
      m_stat_log << now << ", " << m_start_usage.count() << ", "
                 << m_start_rt.count() << ", " << m_synch_usage.count() << ", "
                 << m_synch_rt.count();
      stat_logged = true;
    }
    if (success) {
      for (internal_set::iterator i = m_updates.begin(); m_updates.end() != i;
           ++i) {
        bool echo;

        utils::strand_run(m_graph.strand(),
                          [tl = *i, now] { tl->synchronize(now); });

        Observation const &observ = (*i)->lastObservation(echo);

        // if( echo || is_verbose() || NULL==m_trLog )
        //   syslog(obs)<<observ;
        if (NULL != m_trLog)
          m_trLog->observation(observ);
      }
      m_updates.clear();
    }
    m_obsTick = m_obsTick + 1;

    if (!stat_logged) {
      m_stat_log << getCurrentTick() << ", " << m_synch_usage.count() << ", "
                 << m_synch_rt.count();
    }
    return success;
  } catch (utils::Exception const &e) {
    syslog(error) << "Exception caught: " << e;
  } catch (std::exception const &se) {
    syslog(error) << "C++ exception caught: " << se.what();
  } catch (...) {
    syslog(error) << "Unknown exception caught.";
  }
  syslog(error) << "Failed to synchronize.";
  return false;
}

void TeleoReactor::step() {
  if (NULL != m_trLog)
    m_trLog->step();
  stat_clock::duration delta;
  rt_clock::duration delta_rt;

  {
    utils::chronograph<rt_clock> rt_chron(delta_rt);
    utils::chronograph<stat_clock> stat_chron(delta);
    resume();
  }
  m_deliberation_usage += delta;
  m_delib_rt += delta_rt;
  m_nSteps += 1;
  m_tick_steps += 1;
}

void TeleoReactor::use(TREX::utils::Symbol const &timeline, bool control,
                       bool plan_listen) {
  details::transaction_flags flag; // initialize all the flags to 0
  flag.set(0, control);            // update the control flag
  flag.set(1, plan_listen);        // update the plan_listen flag

  utils::strand_run(m_graph.strand(), [this, timeline, flag] {
    if (!m_graph.subscribe(this, timeline, flag)) {
      if (internal_sync(timeline))
        syslog(warn) << "External declaration of the Internal timeline \""
                     << timeline << "\"";
      else
        syslog(warn) << "Multiple External declarations of timeline \""
                     << timeline << "\"";
    }
  });
}

void TeleoReactor::provide(TREX::utils::Symbol const &timeline,
                           bool controllable, bool publish) {
  details::transaction_flags flag;
  flag.set(0, controllable);
  flag.set(1, publish);

  utils::strand_run(m_graph.strand(), [this, timeline, flag] {
    if (!m_graph.assign(this, timeline, flag)) {
      if (internal_sync(timeline)) {
        syslog(warn) << "Promoted \"" << timeline
                     << "\" from External to Internal with rights "
                     << details::access_str(flag.test(0), flag.test(1));
      }
    }
  });
}

void TeleoReactor::tr_info(std::string const &msg) {
  if (NULL != m_trLog) {
    m_trLog->comment(msg);
  }
}

bool TeleoReactor::unuse(TREX::utils::Symbol const &timeline) {
  return utils::strand_run(m_graph.strand(), [=]() -> bool {
    if (auto i = m_externals.find(timeline); m_externals.end() != i) {
      Relation r{i->first};
      r.unsubscribe();
      return true;
    }
    return false;
  });
}

bool TeleoReactor::unprovide(TREX::utils::Symbol const &timeline) {
  return utils::strand_run(m_graph.strand(), [=]() -> bool {
    if (auto i = m_internals.find(timeline); m_internals.end() != i) {
      (*i)->unassign(getCurrentTick());
      return true;
    }
    return false;
  });
}

void TeleoReactor::isolate(bool failed) {
  if (NULL != m_trLog && failed) {
    Logger *tmp = NULL;
    std::swap(tmp, m_trLog);
    tmp->failed();
    delete tmp;
  }
  clear_internals();
  clear_externals();
}

void TeleoReactor::clear_internals() {
  utils::strand_run(m_graph.strand(), [this] {
    while (!m_internals.empty()) {
      m_internals.front()->unassign(getCurrentTick());
    }
  });
}

void TeleoReactor::clear_externals() {
  utils::strand_run(m_graph.strand(), [this] {
    while (!m_externals.empty()) {
      Relation r = m_externals.begin()->first;
      r.unsubscribe();
    }
  });
}

void TeleoReactor::assigned(details::timeline *tl) {
  m_internals.insert(tl);
  if (is_verbose())
    syslog(null, info) << "Declared \"" << tl->name() << "\" with rights "
                       << tl->rights() << ".";
  if (NULL != m_trLog) {
    m_trLog->provide(tl->name(), tl->accept_goals(), tl->publish_plan());
  }
  for (graph::listen_set::const_iterator i = m_graph.m_listeners.begin();
       m_graph.m_listeners.end() != i; ++i)
    (*i)->declared(*tl);
}

void TeleoReactor::unassigned(details::timeline *tl) {
  internal_set::iterator i = m_internals.find(tl);
  m_internals.erase(i);
  if (is_verbose())
    syslog(null, info) << "Undeclared \"" << tl->name() << "\".";
  if (NULL != m_trLog) {
    m_trLog->unprovide(tl->name());
  }
  for (graph::listen_set::const_iterator i = m_graph.m_listeners.begin();
       m_graph.m_listeners.end() != i; ++i)
    (*i)->undeclared(*tl);
}

void TeleoReactor::subscribed(Relation const &r) {
  external_set::value_type tmp;
  tmp.first = r;
  m_externals.insert(tmp);
  latency_updated(0, r.latency());
  if (is_verbose())
    syslog(null, info) << "Subscribed to \"" << r.name() << "\" with rights "
                       << r.rights() << '.';
  if (NULL != m_trLog) {
    m_trLog->use(r.name(), r.accept_goals(), r.accept_plan_tokens());
  }
  for (graph::listen_set::const_iterator i = m_graph.m_listeners.begin();
       m_graph.m_listeners.end() != i; ++i)
    (*i)->connected(r);
}

void TeleoReactor::unsubscribed(Relation const &r) {
  external_set::iterator i = m_externals.find(Relation::get_id(r));
  // No need to control that i is valid
  //    - this call comes from timeline::unsubscribe so
  //      which calls it only to the client of r -> me
  if (!i->second.empty()) {
    latency_updated(r.latency(), 0);
  }
  // remove this relation
  m_externals.erase(i);
  if (is_verbose())
    syslog(null, info) << "Unsubscribed from \"" << r.name() << "\".";
  if (NULL != m_trLog) {
    m_trLog->unuse(r.name());
  }
  for (graph::listen_set::const_iterator i = m_graph.m_listeners.begin();
       m_graph.m_listeners.end() != i; ++i)
    (*i)->disconnected(r);
}

void TeleoReactor::latency_updated(TICK old_l, TICK new_l) {
  TICK prev = m_maxDelay;

  if (new_l > m_maxDelay)
    m_maxDelay = new_l;
  else if (old_l == m_maxDelay) {
    m_maxDelay = new_l;
    for (details::active_external i(ext_begin(), ext_end()), endi(ext_end());
         endi != i; ++i)
      m_maxDelay = std::max(m_maxDelay, i->latency());
  }
  if (m_maxDelay != prev) {
    // It may be anoying on the long run but for now I will log when this
    // exec latency changes
    syslog(info) << " Execution latency updated from " << prev << " to "
                 << m_maxDelay;
    // Notify all the reactors that depend on me
    for (internal_set::iterator i = m_internals.begin(); m_internals.end() != i;
         ++i)
      (*i)->latency_update(getLatency() + prev);
  }
}

void TeleoReactor::unblock(Symbol const &name) {
  details::external e = find_external(name);
  if (e.active())
    e.unblock();
}

/*
 * class TREX::transaction::TeleoReactor::Logger
 */

// structors

TeleoReactor::Logger::Logger(std::string const &dest,
                             boost::asio::io_service &io)
    : m_strand(io), m_file(io, dest) {
  m_flags.set(header);
  boost::asio::post(m_strand,
                    [this] { direct_write("<Log>\n <header>", true); });
}

TeleoReactor::Logger::~Logger() {
  // std::cerr<<"Destroy logger "<<std::endl;
  // std::cerr<<" - schedulle : close potential tick tag"<<std::endl;
  utils::strand_run(m_strand, [this] {
    close_tick();
    direct_write("</Log>", true);
  });
  // std::cerr<<" - close the file"<<std::endl;
  m_file.close();
}

// interface

void TeleoReactor::Logger::comment(std::string const &msg) {
  boost::asio::post(m_strand,
                    [=] { direct_write("<!-- " + msg + " -->", true); });
}

void TeleoReactor::Logger::init(TICK val) {
  boost::asio::post(m_strand, [=] { set_tick(val, in_init); });
}

void TeleoReactor::Logger::newTick(TICK val) {
  boost::asio::post(m_strand, [=] { set_tick(val, in_new_tick); });
}

void TeleoReactor::Logger::synchronize() {
  boost::asio::post(m_strand, [=] { set_phase(in_synchronize); });
}

void TeleoReactor::Logger::failed() {
  post_event([this] { direct_write("   <failed/>", true); });
}

void TeleoReactor::Logger::has_work() {
  boost::asio::post(m_strand, [=] { set_phase(in_work); });
}

void TeleoReactor::Logger::step() {
  boost::asio::post(m_strand, [=] { set_phase(in_step); });
}

void TeleoReactor::Logger::work(bool ret) {
  std::ostringstream oss;
  oss << "   <work value=\"" << ret << "\" />";

  post_event([this, msg = oss.str()] { direct_write(msg, true); });
}

void TeleoReactor::Logger::provide(Symbol const &name, bool goals, bool plan) {
  std::ostringstream oss;
  oss << "   <provide name=\"" << name << "\" goals=\"" << goals << "\" plan=\""
      << plan << "\" />";
  post_event([this, msg = oss.str()] { direct_write(msg, true); });
}

void TeleoReactor::Logger::unprovide(Symbol const &name) {
  std::ostringstream oss;
  oss << "   <unprovide name=\"" << name << "\" />";
  post_event([this, msg = oss.str()] { direct_write(msg, true); });
}

void TeleoReactor::Logger::use(Symbol const &name, bool goals, bool plan) {
  std::ostringstream oss;
  oss << "   <use name=\"" << name << "\" goals=\"" << goals << "\" plan=\""
      << plan << "\" />";
  post_event([this, msg = oss.str()] { direct_write(msg, true); });
}

void TeleoReactor::Logger::unuse(Symbol const &name) {
  std::ostringstream oss;
  oss << "   <unuse name=\"" << name << "\" />";
  post_event([this, msg = oss.str()] { direct_write(msg, true); });
}

void TeleoReactor::Logger::latency_updated(TICK val) {
  std::ostringstream oss;
  oss << "   <latency value=\"" << val << "\"/>";
  post_event([this, msg = oss.str()] { direct_write(msg, true); });
}

void TeleoReactor::Logger::horizon_updated(TICK val) {
  std::ostringstream oss;
  oss << "   <horizon value=\"" << val << "\"/>";
  post_event([this, msg = oss.str()] { direct_write(msg, true); });
}

void TeleoReactor::Logger::observation(Observation const &o) {
  post_event([=] {
    utils::async_ofstream::entry e = m_file.new_entry();
    o.to_xml(e.stream()) << '\n';
  });
}

void TeleoReactor::Logger::request(goal_id const &goal) {
  post_event([=] { goal_event("request", goal, true); });
}

void TeleoReactor::Logger::recall(goal_id const &goal) {
  post_event([=] { goal_event("recall", goal, true); });
}

void TeleoReactor::Logger::notifyPlan(goal_id const &tok) {
  post_event([=] { goal_event("token", tok, true); });
}

void TeleoReactor::Logger::cancelPlan(goal_id const &tok) {
  post_event([=] { goal_event("cancel", tok, false); });
}

// asio methods


void TeleoReactor::Logger::goal_event(std::string tag, goal_id g, bool full) {
  m_file << "   <" << tag << " id=\"" << g << "\" ";
  if (full)
    g->to_xml(m_file << ">\n") << "\n   </" << tag << ">\n";
  else
    direct_write("/>", true);
}

void TeleoReactor::Logger::set_tick(TICK val,
                                    TeleoReactor::Logger::tick_phase p) {
  close_tick();
  m_current = val;
  m_phase = p;
  m_flags.set(tick);
  m_flags.set(in_phase);
}

void TeleoReactor::Logger::set_phase(TeleoReactor::Logger::tick_phase p) {
  close_phase();
  m_phase = p;
  m_flags.set(in_phase);
}

void TeleoReactor::Logger::open_phase() {
  if (m_flags.test(in_phase) && !m_flags.test(has_data)) {
    open_tick();
    switch (m_phase) {
    case in_init:
      direct_write("  <init>", true);
      break;
    case in_new_tick:
      direct_write("  <start>", true);
      break;
    case in_synchronize:
      direct_write("  <synchronize>", true);
      break;
    case in_work:
      direct_write("  <has_work>", true);
      break;
    case in_step:
      direct_write("  <step>", true);
      break;
    default:
      direct_write("  <unknown>", true);
    }
    m_flags.set(has_data);
  }
}

void TeleoReactor::Logger::close_phase() {
  if (m_flags.test(in_phase)) {
    if (m_flags.test(has_data)) {
      switch (m_phase) {
      case in_init:
        direct_write("  </init>", true);
        break;
      case in_new_tick:
        direct_write("  </start>", true);
        break;
      case in_synchronize:
        direct_write("  </synchronize>", true);
        break;
      case in_work:
        direct_write("  </has_work>", true);
        break;
      case in_step:
        direct_write("  </step>", true);
        break;
      default:
        direct_write("  </unknown>", true);
      }
      m_flags.reset(has_data);
    }
    m_flags.reset(in_phase);
  }
}

void TeleoReactor::Logger::open_tick() {
  if (m_flags.test(tick) && !m_flags.test(tick_opened)) {
    m_file << " <tick value=\"" << m_current << "\">";
    m_flags.set(tick_opened);
  }
}

void TeleoReactor::Logger::close_tick() {
  if (m_flags.test(tick)) {
    if (m_flags.test(tick_opened)) {
      close_phase();
      direct_write(" </tick>", true);
      // std::flush(m_file); // Flush the buffer at every tick
    }
  } else if (m_flags.test(header)) {
    direct_write(" </header>", true);
    m_flags.reset(header);
    m_flags.reset(in_phase);
  }
  m_flags.reset(tick);
  m_flags.reset(tick_opened);
}

void TeleoReactor::Logger::direct_write(std::string const &content, bool nl) {
  utils::async_ofstream::entry e = m_file.new_entry();

  e.stream().write(content.c_str(), content.length());
  if (nl)
    e.stream().put('\n');
}
