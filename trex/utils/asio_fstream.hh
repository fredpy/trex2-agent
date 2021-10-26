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
#ifndef H_trex_utils_asio_fstream
# define H_trex_utils_asio_fstream

# include "SharedVar.hh"
# include "asio_runner.hh"
# include <boost/iostreams/stream.hpp>
# include <boost/asio/io_context_strand.hpp>

# include <string>
# include <memory>
# include <fstream>

namespace TREX {
  namespace utils {
    
    template<class Worker>
    class stranded_service :public boost::asio::io_context::service {
    public:
      static boost::asio::io_context::id id;
      typedef std::shared_ptr<boost::asio::io_context::work> work_ref;
      
      explicit stranded_service(boost::asio::io_context &io)
      :boost::asio::io_context::service(io),m_strand(io) {}
      
      ~stranded_service() {
        shutdown_service();
      }
      
      template<typename Handler>
      void post(Handler const &fn) {
        m_strand.post(fn);
      }
      template<typename Handler>
      void dispatch(Handler const &fn) {
        m_strand.dispatch(fn);
      }
      
      work_ref reserve() {
        work_ref ret = m_work.lock();
        if( !ret ) {
          auto &io = get_io_context();
          ret = std::make_shared<boost::asio::io_context::work>(std::ref(io));
          m_work = ret;
        }
        return ret;
      }
      
      template<typename Handler>
      void async_reserve(Handler fn) {
        boost::function<void (work_ref)> handle(fn);
        post(boost::bind(&stranded_service::reserve_sync, this, handle));
      }
      
      void reserve_sync(boost::function<void (work_ref)> handle) {
        handle(reserve());
      }
      
    private:
      void shutdown_service() {}
      
      std::weak_ptr<boost::asio::io_context::work> m_work;
      boost::asio::io_context::strand m_strand;
    };
    
    template<class Worker>
    boost::asio::io_context::id stranded_service<Worker>::id;

    
    /** @brief Asynchronous text output stream
     *
     * This class implements a text output stream that will do write operations
     * asynchronously. All the write operations of the different instances are 
     * manage by a single strand service eensuring that all the ourtputs are 
     * written in a sequence with limited risk of IO block in the main thread.
     */
    class async_ofstream  {
      class pimpl;
    public:
      typedef stranded_service<pimpl> service;
      
      explicit async_ofstream(boost::asio::io_context &service)
      :m_io(service) {}
      async_ofstream(boost::asio::io_context &service, std::string const &fname)
      :m_io(service) {
        open(fname);
      }
      
      ~async_ofstream() {}
      
      void open(std::string const &fname);
      void close();
      bool is_open() const {
        return NULL!=m_impl.get();
      }
      
      class entry;
      
      class entry_sink {
      public:
        typedef std::string::value_type     char_type;
        typedef boost::iostreams::sink_tag category;
        
        entry_sink();
        entry_sink(entry_sink const &other);
        ~entry_sink() {
          flush();
        }
        
        std::streamsize write(char_type const *s, std::streamsize n);

      private:
        explicit entry_sink(std::shared_ptr<pimpl> const &dest);
        void flush();
        
        mutable std::shared_ptr<pimpl> m_dest;
        mutable std::string       m_cache;
        
        friend class entry;
      };
      
      typedef boost::iostreams::stream<entry_sink> entry_impl;
      
      class entry {
      public:
        entry(entry const &other);
        ~entry() {}
        
        std::ostream &stream();
        operator std::ostream &() {
          return stream();
        }
        template<typename Ty>
        std::ostream &operator<<(Ty val) {
          return stream()<<val;
        }
        
        std::ostream &write(entry_sink::char_type const *s, std::streamsize n) {
          return stream().write(s, n);
        }
        
        void flush();
        
      private:
        explicit entry(std::shared_ptr<pimpl> const &dest);
        
        mutable std::unique_ptr<entry_impl> m_out;
        
        friend class async_ofstream;
        entry() =delete;
      };
    
      entry new_entry();
      template<typename Ty>
      entry operator<<(Ty val) {
        entry tmp = new_entry();
        tmp<<val;
        return tmp;
      }
      
    private:
      boost::asio::io_context &m_io;
      std::shared_ptr<pimpl>        m_impl;
    };
    
    
    class async_buffer_sink {
    public:
      typedef async_ofstream::entry_sink::char_type char_type;
      typedef boost::iostreams::sink_tag            category;

      explicit async_buffer_sink(async_ofstream &dest)
      :m_dest(dest.new_entry()) {}
      ~async_buffer_sink() {}
      
      std::streamsize write(char_type const *s, std::streamsize n);
      
    private:
      async_ofstream::entry m_dest;
    };
    
    typedef boost::iostreams::stream<async_buffer_sink> async_proxy;
  
  } // TREX::utils
} // TREX

#endif // H_trex_utils_asio_fstream
