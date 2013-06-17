/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, MBARI.
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
#ifndef H_trex_transaction_graph_impl
# define H_trex_transaction_graph_impl

# include <trex/utils/LogManager.hh>

# include <boost/enable_shared_from_this.hpp>
# include <boost/asio/strand.hpp>

namespace TREX {
  namespace transaction {
    namespace details {
      
      /** @breif Graph internal implementation
       *
       * This class handles the implementation details of the transaction graph.
       * It maintins the graph states along with the control of its modification
       * and access through a specific strabdnd which can be seen as a virtual 
       * thread.
       *
       * @note This class is not part of trex public API and is not accessible 
       * when trex is installed.
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup transaction
       */
      class graph_impl :boost::noncopyable,
      public boost::enable_shared_from_this<graph_impl> {
      public:
        typedef utils::log::entry::date_type date_type;
        
        /** @brief Default constructor
         *
         * Create a nameless graph
         */
        graph_impl();
        /** @brief Constructor 
         *
         * @param[in] name A symbolic name
         *
         * Create a graph named @p name
         */
        explicit graph_impl(utils::Symbol const &name);
        /** @brief Destructor 
         */
        ~graph_impl();
        
        /** @brief graph name
         * 
         * @return The name of the graph
         */
        utils::Symbol const &name() const {
          return m_name;
        }
        /** @brief set graph name
         *
         * @param[in] name A symbolic name
         *
         * Change the graph names to @p name
         */
        void name(utils::Symbol const &name) {
          m_name = name;
        }
        /** @brief Update date
         *
         * @param[in] d A date value
         *
         * Update the datum of the graph to @p d
         */
        void set_date(date_type const &d);
        /** @brief Get date
         *
         * @param[in] fast Access flag
         *
         * Get the date associated to the graph if any.
         * The @p fast flag indicates whether the access to this date should 
         * be quick or potentially slower wbut with the added insurance that 
         * any pending date update will be resolved before reading it.
         *
         * @return The date associated to this graph
         */
        boost::optional<date_type> get_date(bool fast=false) const;
        
        /** @brief Log method
         *
         * @param[in] ctx A symbol indicating the local producer
         * @param[in] kind A symbol indicating the type of message (error, warn,...)
         *
         * Create a new log stream for the context @p ctx, associated message
         * type @p kind and datum associated to current graph date
         *
         * @return A stream that can receive the log message
         */
        utils::log::stream syslog(utils::Symbol const &ctx,
                                  utils::Symbol const &kind) const;
        /** @brief graph log manager
         *
         * @return The log manager for this graph
         *
         * @note As of today the LogManager is handled as a singleton. Therefore 
         * all the graphs within one process will share the same manager. Still 
         * this could change in the future, therefore it is better to access the 
         * log manager through this method.
         */
        utils::LogManager &manager() const {
          return *m_log;
        }
        /** @bief Transaction strand
         *
         * @return The strand associated to this graph and used to any 
         * access or update oof this grsah structure
         */
        boost::asio::strand &strand() const {
          return *m_strand;
        }
        
      private:
        void set_date_sync(date_type date);
        
        /** @brief graph date local storage
         *
         * The attribute where the graph date is locally stored. While this 
         * attribute is updated through the starnd it is still mutex protected
         * in order to allow quick access when Hasving Accurate date is not 
         * critical
         */
        mutable utils::SharedVar< boost::optional<date_type> > m_date;
        /** @brief graph name local storage
         *
         * The attrobute that stroes the graph assocaited name. This attribute 
         * is not thread protected as it is assumed that it won't be updated 
         * during the agent execution
         */
        utils::Symbol                          m_name;
        /** @brief Log manager
         *
         * A reference to the global LogManager singleton.
         */
        utils::SingletonUse<utils::LogManager> m_log;
        /** @brief Acess/update strand
         *
         * The strand which is used by this graph for accesses and updates
         * of its structure
         */
        UNIQ_PTR<boost::asio::strand>          m_strand;
      }; // TREX::transaction::details::graph_impl
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_graph_impl