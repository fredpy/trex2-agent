/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#ifndef H_trex_transaction_clock_impl
# define H_trex_transaction_clock_impl

# include "../bits/transaction_fwd.hh"

# include <boost/thread/shared_mutex.hpp>

namespace TREX {
  namespace transaction {
    namespace details {
      
      /** @brief tick value holder
       *
       * This class is were the current tick value of the agent
       * is stored and accessed. 
       *
       * @author Frederic Py <fredpy@gmail.com>
       */
      class clock :boost::noncopyable,public ENABLE_SHARED_FROM_THIS<clock> {
        /** @brief Underlying mutex used
         *
         * This is the type of mutex used by this class to protect access 
         * to its attributes. This class uses @c boost::shared_mutex as 
         * this mutex allows to implement multiple concurrent reads with 
         * a single exclusive write. 
         *
         * As a result while the update of the clock values is blocking 
         * any other access, multiple concurrent processes can read it
         * concurrently while not being updated. This limits race 
         * conditions between multiple threads trying to get the current 
         * tick.
         */
        typedef boost::shared_mutex mutex_type;
      
      public:
        /** @brief Basic type for a date
         *
         * The type used by this class to represent a date.
         */
        typedef utils::log::entry::date_type date_type;
        
        /** @brief Constructor
         *
         * @param[in] io The service managing clock updates
         *
         * Create a new instance of this class and assign its updates 
         * to a strand associated to @p io
         */
        explicit clock(boost::asio::io_service &io);
        /** @brief Destructor */
        ~clock();
        
        /** @brief Check if started
         *
         * Check if the clock started flag is set to true
         *
         * @retval true if the clock is marked as started
         * @retval false otherwise
         * 
         * @sa clock::set_started(bool)
         */
        bool      started()  const;

        /** @brief Get clock date
         *
         * Get the date value started by this clock if any
         *
         * @retrun an optional date. If the optional is not set than 
         *         this clock data has yet to be updated, otherwise
         *         it is the value of the date
         *
         * @sa clock::set_date(date_type const &)
         */
        boost::optional<date_type> get_date() const {
          return read_date();
        }
        
        /** @brief Update clock date
         *
         * @param[in] val A date
         *
         * Attempt to set the date to @p val.
         *
         * @note the update will be made only if @p val is not less or equal
         *       to the current date
         * @post An attempt to update the date to @p val is schedulled in
         *       another thread
         *
         * @sa clock::set_date_sync(date_type)
         * @sa clock::get_date() const
         */
        void set_date(date_type const &val);
        /** @brief Update clock started flag
         *
         * @param[in] flag suggest started value
         *
         * request the updated of this clock started flag to @p flag
         *
         * @post An update of started state to @p flag is schedulled in
         *       another thread
         *
         * @sa clock::set_start_sync(bool)
         * @sa clock::started() const
         */
        void set_started(bool flag = true);
        
      private:
        boost::asio::strand        m_strand;
        mutable mutex_type         m_mutex;
        
        bool                       m_started;
        boost::optional<date_type> m_date;
        
        /** @brief Get current date
         *
         * @return the latest date value
         *
         * @note This method handle the access to date with 
         *       the proper access control allowing concurrent 
         *       reads but blocked by a single write
         */
        boost::optional<date_type> read_date() const;
        
        /** @brief Update clock date
         *
         * @param[in] date proposed date value
         *
         * This method will update this clock date to @p date only if
         * @p date is no less or equalt to the currently stored date.
         *
         * @note this method is executed by a specific strand 
         * @note this method handle write access to the date and 
         *       block any other operations until completion
         *
         * @post the date of this clock is greater or equal to @p date
         */
        void set_date_sync(date_type date);
        /** @brief Update clock start flag
         *
         * @param[in] falg proposed started date
         *
         * This method  update this clock started flag to @p flag
         *
         * @note this method is executed by a specific strand
         * @note this method handle write access to the started flag and
         *       block any other operations until completion
         *
         * @post the started of this clock is equal to @p flag
         */
        void set_start_sync(bool flag);
        
        // following methods have puposedly no code
        clock() DELETED;
      }; // TREX::transaction::details::clock
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_clock_impl
