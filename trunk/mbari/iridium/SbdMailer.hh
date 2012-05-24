#ifndef H_SbdMailer 
# define H_SbdMailer

# include <trex/utils/LogManager.hh>

# include <Poco/Net/SMTPClientSession.h>

# include <set>

namespace mbari {
  namespace iridium {
    
    class SbdMailer {
    public:
      typedef Poco::Net::SMTPClientSession::LoginMethod LoginMethod;


      SbdMailer(std::string const &host, 
		Poco::UInt16 port = Poco::Net::SMTPClientSession::SMTP_PORT);
      ~SbdMailer();

      void login();
      void login(LoginMethod method, std::string const &user_name, 
		 std::string const &password);
      void close();

      std::string const &sender() const {
	return m_sender;
      }
      void set_sender(std::string const &sender) {
	m_sender = sender;
      }

      void add_recipient(std::string const &address);
      size_t recipients() const;
      bool has_recipient(std::string const &address) const;
      void remove_recipient(std::string const &address);
      
      void send(std::string imei, char const *data, size_t size,
                std::string const &comment = "");

      static std::string const s_iridium_address;

    private:
      Poco::Net::SMTPClientSession m_server;
      std::set<std::string> m_recipients;
      std::string m_sender;

      static std::string const s_sbd_ext;
      static size_t counter;
      
      static std::string new_file();


      TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
    };

  }
}

#endif // H_SbdMailer
