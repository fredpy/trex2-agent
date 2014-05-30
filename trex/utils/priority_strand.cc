#include "priority_strand.hh"

using namespace TREX::utils;
namespace asio=boost::asio;

/*
 * class TREX::utils::priority_strand::task
 */

bool priority_strand::task::operator< (priority_strand::task const &other) const {
  return other.m_level && ( !m_level || *other.m_level<*m_level );
}

/*
 * struct TREX::utils::priority_strand::task_cmp
 */

bool priority_strand::tsk_cmp::operator()(priority_strand::task *a,
                                          priority_strand::task *b) const {
  return a->operator<(*b);
}

/*
 * class TREX::utils::priority_strand
 */

// structors

priority_strand::priority_strand(asio::io_service &io)
:m_strand(io) {}

priority_strand::~priority_strand() {}

// manipulators

void priority_strand::enqueue(priority_strand::task *t) {
  m_strand.dispatch(boost::bind(&priority_strand::enqueue_sync, this, t));
}

// strand protected methods

void priority_strand::enqueue_sync(priority_strand::task *t) {
  m_tasks.push(t);
  m_strand.dispatch(boost::bind(&priority_strand::dequeue_sync, this));
}

void priority_strand::dequeue_sync() {
  task *nxt = m_tasks.top();
  m_tasks.pop();
  nxt->execute();
  delete nxt;
}
