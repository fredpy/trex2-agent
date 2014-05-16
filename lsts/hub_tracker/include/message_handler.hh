#ifndef H_message_handler 
# define H_message_handler

# include <string>
# include <vector>
# include <map>

# include <boost/shared_ptr.hpp>
# include <boost/property_tree/ptree.hpp>

# include <DUNE/IMC/IridiumMessageDefinitions.hpp>
# include <DUNE/Network/FragmentedMessage.hpp>

namespace lsts_hub {

  class message_handler :boost::noncopyable {
  public:
    typedef boost::shared_ptr<DUNE::IMC::Message> return_type;
    typedef std::string const &                   argument_type;

    message_handler() {}
    ~message_handler();

    return_type operator()(argument_type msg) {
      return process(msg);
    }
    /** @brief Process a message
     *
     * @param[in] content the message content as an hex encoded string
     *
     * Parse content as a dune message. If the parsing resulted on 
     * a message part the message is added to the cache of this class 
     * unless its addition resulted on a completed message. 
     *
     * @retval A pointer to the resulting message or a null pointer if 
     * the message is not complete yet
     */
    return_type process(argument_type msg);

  private:
    typedef DUNE::Network::FragmentedMessage defrag_type;
    typedef std::map<uint8_t, boost::shared_ptr<defrag_type> > parts_cache;

    parts_cache m_pending;
    
    /** @brief Cache and process message parts
     *
     * @param[in] part a message part
     *
     * Add @p part in the corresponding multipart message and process the
     * result. If this addition result on a full message it then generate 
     * the new message and remove the parts from the cache.
     *
     * @return A null pointer if there's still missing part for the message 
     *   associated to @p part or pointer to the new full message if @p part
     *   was the last missing part for this message
     */
    return_type process_part(DUNE::IMC::MessagePart *part);
    /** @brief Convert a character to its haxdeciaml value
     *
     * @param[in] v A character
     *
     * Parse @p v as an hexadecimal digit 
     *
     * @return the interger value for @p v 
     *
     * @throw std::runtime_error @p v do not represent an hexadecimal code
     */
    static uint8_t hex_char(char c);
    
    /** @brief Text message hex parsing
     *
     * @param[in] val A message encoded as an hex string
     *
     * Parse @p val as a string encoding of an hexadecimal value. The message 
     * can be as long as needed and just need to be exclusively text represenation 
     * of hexadecmial digits (i.e. [0-9] or [a-fA-F])
     *
     * @return  a vector representing @p val as binary data
     *
     * @throw std::runtime_error The format of @p val is not valid
     */
    static std::vector<uint8_t> decode_hex(std::string const &val);
  }; // lsts_hub::message_handler
  
  /** @brief Parse and process a TrexOperation message
   *
   * @param[in] ptr A pointer to the message
   *
   * @pre ptr refers to a TrexOperation message
   *
   * @return a property_tree containing relevant information
   * from the message dfor visualization
   */
  boost::property_tree::ptree 
  compact_json(message_handler::return_type const &ptr);

} // lsts_hub

#endif // H_message_handler
