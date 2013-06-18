#ifndef H_trex_utils_asio_signal_base
# define H_trex_utils_asio_signal_base

# include <boost/asio/io_service.hpp>
# include <boost/signals2/signal.hpp>

# include "../platform/cpp11_deleted.hh"

namespace TREX {
  namespace utils {
    namespace details {
      
      template<typename Signature>
      class asio_signal_base :boost::noncopyable {
      private:
        typedef boost::signals2::signal<Signature> signal_base;
        
        boost::asio::io_service &m_service;
      public:
        typedef typename signal_base::signature_type     signature_type;
        typedef typename signal_base::result_type        result_type;
        typedef typename signal_base::slot_type          slot_type;
        typedef typename signal_base::extended_slot_type extended_slot_type;
        
        typedef boost::signals2::connection              connection;
        
        template<unsigned N>
        class arg :public signal_base::template arg<N> {};
        
        typedef boost::asio::io_service::strand          strand;
        
        explicit asio_signal_base(boost::asio::io_service &io)
        :m_service(io) {}
        
        boost::asio::io_service &service() {
          return m_service;
        }
        strand new_strand() {
          return strand(service());
        }
        
        bool empty() const {
          return m_signal.empty();
        }
        size_t num_slots() const {
          return m_signal.num_slots();
        }
        
        connection connect(slot_type const &cb) {
          return m_signal.connect(wrap(service(), cb));
        }
        connection connect_extended(extended_slot_type const &cb) {
          return m_signal.connect_extended(wrap(service(), cb));
        }
        
        connection strand_connect(strand &s, slot_type const &cb) {
          return m_signal.connect(wrap(s,cb));
        }
        connection strand_connect(slot_type const &cb) {
          strand s=new_strand();
          return strand_connect(s, cb);
        }
        
        connection strand_connect_extended(strand &s, extended_slot_type const &cb) {
          return m_signal.connect_extended(wrap(s,cb));
        }
        connection strand_connect_extended(extended_slot_type const &cb) {
          strand s=new_strand();
          return strand_connect_extended(s, cb);
        }
      protected:
        template<typename Service, class Slot>
        Slot wrap(Service &s, Slot const &slot) {
          return s.wrap(slot);
        }
      
        signal_base              m_signal;
      private:
        asio_signal_base() DELETED;
      }; // TREX::utils::details::asio_signal_base
      
    } // TREX::utils::details
  } // TREX::utils
} // TREX

#endif // H_trex_utils_asio_signal_base