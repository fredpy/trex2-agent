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
/** @file "trex/utils/asio_signal.hh"
 * @brief asynchronous signal handling utilities
 *
 * This file defines an experimental class for both using boost preprocessing 
 * macros for generic class specialization and dealing with signal that are handled 
 * asynchronously.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */ 
#ifndef H_trex_utils_asio_signal
# define H_trex_utils_asio_signal

# include "asio_signal_fwd.hh"

namespace TREX {
  namespace utils {
    
# ifndef DOXYGEN
    
#  include "bits/asio_signal_iter.hh"

# else // DOXYGEN
       // Note: all the code below is purelly for documentation puspose and
       //      not meant to be compiled
  
    /** @brief Asynchronous signal management class
     *
     * @tparam Signature A void function signature (e.g. @c void (int,std::string))
     *
     * This template class implements signal management with asyncrhonous execution
     * of the signal handlers.
     *
     * The handler for this signal are functors with no returned value and a
     * signature compatible with @p Signature. When a signal is emitted through
     * this class the execution of the handlers that were connected to it will be
     * executed asynchronously by the asio service that is attched to this class.
     * This allow to dispatch the signal quickly on the signal  producing class when
     * its handling by the different handler connected is deported to the thread(s)
     * that execute our service. By doing so the handling of this signal can easily
     * be ran in a multithreaded program.
     *
     *
     * @pre Signature is a function signature with void as returned value
     *
     * @note While the number of arguments accepted can be set at compilation by
     * defining the macro @c ASIOSIG_MAX_SIZE, by default when generate only for
     * functions with up to 6 arguments.
     *
     * @author Frederic Py
     * @ingroup utils
     */
    template<typename Signature>
    class asio_signal {
    public:
      /** @brief signal handling signature
       *
       * The signature of the functor that can handle these messages
       */
      typedef Signature signature_type;
      /** @brief returned value
       *
       * The value returned by the signal handler. It is set to @c void as we
       * only support signals with no result.
       */
      typedef void      result_type;
      
      /** @brief Nth argument type
       *
       * @tparam N an integer
       *
       * Used to identify the type of the @p N +1th argument of this signal
       */
      template<unsigned N>
      class arg {
        /** @brief type of the argeument
         *
         * The type of the N+1th argument of @p Signature
         */
        typedef computed_type type;
      };
      
      /** @brief argument type
       *
       * Utility typedef to make this class a unary_function
       *
       * @pre Signature has only one argument
       */
      typedef arg<0>::type argument_type;
      /** @brief first argument's type
       *
       * Utility typedef to make this class a binary_function
       *
       * @pre Signature has exactly 2 arguments
       * @sa second_argument_type;
       */
      typedef arg<0>::type first_argument_type;
      /** @brief second argument's type
       *
       * Utility typedef to make this class a binary_function
       *
       * @pre Signature has exactly 2 arguments
       * @sa first_argument_type;
       */
      typedef arg<1>::type second_argument_type;
      
      /** @brief function arity
       *
       * The number of argument for @p Signature
       */
      static unsigned const arity;
      
      /** @brief signal handler execution slot
       *
       * The type used to store and execute the handlers connected to this
       * signal.
       * @sa connect(slot_type const &)
       */
      typedef computed_type slot_type;
      /** @brief slot handler storage
       *
       * The type used for a slot to store the handler it is associated to
       * @sa slot_type
       */
      typedef computed_type slot_function_type;
      /** @brief extended signal handler execution slot
       *
       * The type used to store and execute the extended handlers connected to this
       * signal. An extended handler accepts an extr first argument as the
       * connection which was used to trigger it. This allow the handler to do
       * extra management on its connection to the signal (such as for example
       * disabling this connection) or know the source
       * of this signal.
       *
       * @sa slot_type
       * @sa connect_extended(extended_slot_type const &)
       */
      typedef computed_type extended_slot_type;
      /** @brief extended slot handler storage
       *
       * The type used for a extended slot to store the handler it is associated to
       * @sa extended_slot_type
       */
      typedef computed_type extended_slot_function_type;
      
      /** @brief Connection descriptor
       *
       * The type used to describe and manage a conection of a signal to an handler
       */
      typedef boost::signals2::connection     connection;
      /** @brief Sequencing service
       *
       * This the type used in order to ensure that one or more handler will never
       * be ran concurrently. Indeed without wrapping the handler in a strand
       * there's no way to ensure that its execution will not occur conncurrently
       * the strand class allow to prevent such situation by serializing all the
       * calls it is associated to.
       *
       * @sa strand_connect
       * @sa strand_connect_extended
       * @sa strand_connect_protected
       */
      typedef boost::asio::io_service::strand strand;
      
      /** @brief Constructor
       *
       * @param[in] io An asio service
       *
       * Create a new signal instance associated to the asio service @p io.
       * The handling of events from this signal will be executed
       * by @p io
       */
      explicit asio_signal(boost::asio::io_service &io);
      /** @brief Destructor */
      ~asio_signal();
      
      /** @brief signal execution service
       *
       * @return  the asio service used to execute the handling of this signal
       */
      boost::asio::io_service &service();
      /** @brief New starnd creation
       *
       * Create a new starnd which can then be used to serialize some signal
       * handlers.
       *
       * @return A new starnd executed by @c service()
       */
      strand new_strand();
      
      /** @brief Signal production
       *
       * @param the arguments of the signal
       *
       * @pre the number of types of argment p[assed is identical to the ones
       *    from @p Signature
       *
       * Emit a new signal with the associated arguments this signal will then
       * be handled by the slots connected to this signal through the
       * asynchronous service @c service()
       */
      void operator()(...);
      
      /** @brief Check for slots
       *
       * @retval true if no handler connected to this signal
       * @retval false otherwise
       *
       * @sa num_slots() const
       */
      bool empty() const;
      
      /** @brief Count handlers
       *
       * @return the number of slots connected to this signal
       * @sa empty() const
       * @sa connect(slot_type const &)
       */
      size_t num_slots() const;
      
      /** @brief Connect new handler
       *
       * @param[in] cb A new slot
       *
       * Connect the handler @p cb to this signal. @p cb is also wrapped by
       * the signal @c service() so its execution when a new signal is produced
       * will be executed asynchronously
       *
       * @return the connection between this signal and @p cb
       * @sa strand_connect
       */
      connection connect(slot_type const &cb);
      /** @{
       * @brief Serialized connection
       *
       * @param[in] s  a strand (optional)
       * @param[in] cb a new slot
       *
       * connect the handle @p cb to this signal. @p cb is warpped by the strand
       * @p s or a temporary starnd created specifically for this connection.
       * This implies that the execution of @p cb will not be alloeed to run
       * concurrently with any tasks executed by @p s which can be usefull to do
       * operations in @p cb which would noyt be thread safe.
       *
       *
       * @return the connection between this signal and @p cb
       * @sa connect
       * @sa new_strand()
       */
      connection strand_connect(strand &s, slot_type const &cb);
      connection strand_connect(slot_type const &cb);
      /** @} */
      /** @brief Connect new extended signal handler
       *
       * @param[in] cb a new extended slot
       *
       * Connect the hanextnded handler @p cb to this signal. @p cb is also
       * wrapped by the signal @c service() so its execution when a new signal
       * is produced will be executed asynchronously.
       *
       * An extended handler accepts as its first argument -- on top of the
       * signal arguments -- the connection involved. This alow such handler to
       * both idenitify this connection and potentially modify it.
       *
       * @return the connection onnection between this signal and @p cb which will
       * be the first argument passed to  @p cb
       *
       * @sa strand_connect_extended
       */
      connection connect_extended(slot_type const &cb);
      /** @{
       * @brief Serialized extended connection
       *
       * @param[in] s  a strand (optional)
       * @param[in] cb a new extended slot
       *
       * Connect the handle @p cb to this signal. @p cb is warpped by the strand
       * @p s or a temporary starnd created specifically for this connection.
       * This implies that the execution of @p cb will not be alloeed to run
       * concurrently with any tasks executed by @p s which can be usefull to do
       * operations in @p cb which would noyt be thread safe.
       *
       *
       * @return the connection between this signal and @p cb
       * @sa connect_extended
       * @sa new_strand()
       */
      connection strand_connect_extended(strand &s, extended_slot_type const &cb);
      connection strand_connect_extended(extended_slot_type const &cb);
      /** @} */
    };
    
# endif // DOXYGEN

  } // TREX::utils
} // TREX

#endif // H_trex_utils_asio_signal
