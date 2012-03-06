/* -*- C++ -*-
 */
/** @file "RStat.hh"
 *
 * @brief Definition of RStat
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
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
#ifndef H_RStat
# define H_RStat

# include <sys/resource.h>

# include "ErrnoExcept.hh"
# include "TimeUtils.hh"

namespace TREX {
  namespace utils {

    /** @brief Ressource usage stats management class.
     *
     * This class offer a simple and basic access to resource usage
     * statistics for processes.
     *
     * @author Frederic Py <fpy@mbari.org>
     *
     * @note This class is based on POSIX C function @c getrusage. It does
     * not yet offer all the interfaces to access data given by this
     * function.
     * @ingroup utils
     */  
    class RStat {
    public:
      /** @brief stats about who
       *
       * This type is used by RStat to specify what
       * processes are described on statisitics.
       */
      enum who_type {
	/** @brief self statisitics.
	 */
	self = RUSAGE_SELF, 
	/** @brief children statistics.
	 *
	 * @bug As said in the man page of @c getrusage : 
	 * <em>"There is no way to obtain information about
	 * a child process that has not yet terminated."</em>
	 */
	children = RUSAGE_CHILDREN,
	/** @brief Unknown statisitics.
	 *
	 * This occur when you make operation with RStat
	 * instances having different @c who_type.
	 */
	zeroed = 1,
	unknown = 2
      }; // RStat::who_type
    
      /** @brief Constructor.
       *
       * @param who Stats about who information
       *
       * @pre @e who value must be one of RStat::self or
       * RStat::children
       *
       * Create a new insatnce and get the stats for the given
       * @e who. This give a direct snapshot of the process(es).
       *
       * @throw ErrnoExcept A problem occured when getting stats.
       * The most probable cause is a bad value for @e who
       * (errno==EINVAL).
       */
      RStat(who_type who=self);
      /** @brief Destructor */
      ~RStat() {}
    
      /** @brief Get new stats.
       *
       * This method is called to update the stats values of
       * this instance.
       *
       * @pre The who() value must be one of RStat::self or
       * RStat::children
       *
       *  @throw ErrnoExcept A problem occured when getting stats.
       * The most probable cause is a bad value for who()
       * (errno==EINVAL).
       *
       * @sa who() const
       */
      void reset(who_type who=unknown);
    
      /** @brief Type of process stats.
       *
       * @return the type of processes the stats are
       * depicting.
       */
      who_type who() const {
	return _kind;
      }
    
      /** @brief User time
       *
       * @return The amount of time spent on user
       * execution space. The user time correspond
       * to the non system calls.
       *
       * @sa system_time() const
       */
      timeval const &user_time() const {
	return _snapshot.ru_utime;
      }
    
      /** @brief System time
       *
       * @return The amount of time spent on system
       * execution space. The system time correspond
       * to the time spent on system calls.
       *
       * @sa user_time() const
       */
      timeval const &system_time() const {
	return _snapshot.ru_stime;
      }
    
      /** @brief Maximum resident set size.
       *
       * @return the maximum amount of
       * resident memory used.
       *
       * @note It is not clear to right now what is the unity of
       * this value. Indeed, it looks like it is given in term
       * of memory pages. In this case we would probably need to
       * get the page size on current system to convert into bytes.
       * Right now I consider that this size is given in bytes.
       *
       * @todo This value is for resident memory size. It will be
       * @b less or equal to the real amount of memory. I have to
       * find out a way to have the total amount of memory used
       * but didn't found doc about that.  
       *
       * @bug This information is not updated in real-time
       * by the system. The update rate of your OS can be
       * found using the command :
       * @code
       * sysctl kern.clockrate
       * @endcode 
       * The value of @c stathz indicates the frequency
       * of the updates (100Hz under MacOS X.4). Keep this in mind when
       * you read the results.
       */
      long const &max_resident() const {
	return _snapshot.ru_maxrss;
      }
    
      /** @brief Number of page reclaims
       *
       * @return The number of page reclaimed
       */
      long const &n_page_reclaims() const {
	return _snapshot.ru_minflt;
      }
      /** @brief Number of page faults
       *
       * @return The number of page faults
       */
      long const &n_page_faults() const {
	return _snapshot.ru_majflt;
      }
    
      /** @brief Difference between stats.
       *
       * @param other another stats snapshot.
       *
       * @return A new stats snapshot giving the difference
       * between current instance and @e other.
       *
       * @note If the who() value differs form current
       * instance and @e other, then the who() value of
       * the result will be RStat::unknown which prohibit
       * the call to @e reset.
       *
       * @todo It may be preferable to throw an exception
       * if who() types differs.
       */
      RStat operator- (RStat const &other) const;

      /** @brief Addition of stats.
       *
       * @param other another stats snapshot.
       *
       * @return A new stats snapshot giving the sum
       * of current instance and @e other.
       *
       * @note If the who() value differs form current
       * instance and @e other, then the who() value of
       * the result will be RStat::unknown which prohibit
       * the call to @e reset.
       *
       * @todo It may be preferable to throw an exception
       * if who() types differs.
       */
      RStat operator+ (RStat const &other) const;
      /** @brief Multiplication.
       *
       * @param n a factor
       *
       * @return A new stats snapshot which is current
       * instance multiplied by @e n
       */
      RStat operator* (long n) const;
      /** @brief Division.
       *
       * @param n a divider
       *
       * @pre n!=0
       *
       * @return A new stats snapshot which is current
       * instance divided by @e n
       */
      RStat operator/ (long n) const;
    
      /** @brief Long stats description.
       *
       * @param out an output stream
       *
       * This method writes a full descrip[tion of the stats grabbed
       * by this instance into @e out
       *
       * @return @e out after the operation
       *
       * @sa std::ostream &compact_desc(std::ostream &out) const
       */
      std::ostream &long_desc(std::ostream &out) const;
      /** @brief Short stats description.
       *
       * @param out an output stream
       *
       * This method writes a short descrip[tion of the stats grabbed
       * by this instance into @e out. The format of the string written
       * is as follow :
       * @code
       * user_time, system_time, max_resident, n_page_reclaims, n_page_faults
       * @endcode
       *
       * @return @e out after the operation
       *
       * @sa std::ostream &long_desc(std::ostream &) const
       * @sa std::ostream &operation<<(std::ostream &, RStat const &)
       */
      std::ostream &compact_desc(std::ostream &out) const;

      /** @brief Compact stats header descprition */
      static std::string compact_header();

    private:
      /** @brief Kind of the stats */
      who_type _kind;
      /** @brief Stats snapshot */
      rusage   _snapshot;
    
      /** @brief Constructor
       *
       * This constructor is used internnalyy to avoid the call of reset()
       * when creating a new insatnce.
       */
      RStat(who_type who, bool)
	:_kind(who) {}
    }; // RStat

    /** @brief Output write operator
     * @relates RStat
     *
     * @param out An output stream
     * @param stats A stats snapshot
     *
     * Write the compact_desc of @e stats to @e out
     *
     * @return @e out after the operation.
     *
     * @sa std::ostream &RStat::compact_desc(std::ostream &)
     * @ingroup utils
     */
    inline std::ostream &operator<<(std::ostream &out, RStat const &stats) {
      return stats.compact_desc(out);
    }
  
    /** @brief Output write operator
     * @relates RStat::who_type
     *
     * @param out An output stream
     * @param who A stats type
     *
     * Write the @e who in human readable form to @e out
     *
     * @return @e out after the operation.
     * @ingroup utils
     */
    inline std::ostream &operator<<(std::ostream &out, RStat::who_type who) {
      switch( who ) {
      case RStat::self:
	return out<<"self";
      case RStat::children:
	return out<<"children";
      default:
	return out<<"???";
      }
    }

    /** @brief Multiplication
     * @relates RStat
     *
     * @param n factor
     * @param s A statisitic information
     *
     * Multiplies @e s by @e n. This function assure the commutativity
     * of the multiplication.
     *
     * @return The result of the multiplication.
     *
     * @sa RStat RStat::operator* (long n) const
     * @ingroup utils
     */
    inline RStat operator* (long n, RStat const &s) {
      return s*n;
    }

    /** @brief Stop-watch for RStat
     *
     * This class is an helper to measure the delta between two
     * time points. the tow timepoints are given bytthe date of
     * creation and destruction of this class instance and addded
     * on a RStat instance given as attribute.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class RStatLap {
    public:
      /** @brief Constructor
       * @param output Output variable
       * @param who type of data to track
       *
       * Starts the timer on @e who stats. The resulted will be added
       * to @e output when this instance will be destroyed.
       */
      RStatLap(RStat &output, RStat::who_type who)
	:m_output(output), m_start(who) {
      }
      /** @brief Destructor
       *
       * get the delta between current stats and the sats on
       * this instance creation and add the result to the referred @e output
       */
      ~RStatLap() {
	RStat end(m_start.who());
	m_output = m_output+(end-m_start);
      }
      
    private:
      RStat &m_output; //!< Reference to the output variable
      RStat m_start;   //!< Stats collected on this instance creation

      // Following methods have no code in purpose
      RStatLap();
    }; // TREX::utils::RStatLap

  } // TREX::utils
} // TREX

#endif // H_RStat 
