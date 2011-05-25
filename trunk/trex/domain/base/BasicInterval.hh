/* -*- C++ -*- */
/** @file "BasicInterval.hh"
 * @brief Abstract interface for interval domains
 *
 * This file defines the bases class for implementing interval
 * domains in TREX. It offers multiple utilities to have unified
 * access/input/output for this kind of domain.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef H_BasicInterval
# define H_BasicInterval

# include "DomainBase.hh"

namespace TREX {
  namespace transaction {

    /** @brief Interval Domain base class
     *
     * This class provide a unified abstract interface for implementing
     * interval domains. It especially provide a way to ease and unify
     * the access methods and input/output for all the domain of this kind.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    class BasicInterval :public DomainBase {
    public:
      /** @brief Destructor */
      virtual ~BasicInterval() {}

      bool isInterval() const {
	return true;
      }
      bool isEnumerated() const {
	return false;
      }

      virtual bool hasLower()  const =0;
      virtual bool hasUpper()  const =0;

      bool isFull() const {
	return !( hasLower() || hasUpper() );
      }

      std::ostream &toXml(std::ostream &out, size_t tabs) const;

      virtual boost::any getLower() const =0;
      virtual boost::any getUpper() const =0;

      template<class Ty, bool Safe>
      Ty getTypedLower() const {
	if( Safe ) 
	  return TREX::utils::string_cast<Ty>(getStringLower());
	else 
	  return boost::any_cast<Ty>(getLower());
      }
           
      template<class Ty, bool Safe>
      Ty getTypedUpper() const {
	if( Safe ) 
	  return TREX::utils::string_cast<Ty>(getStringUpper());
	else 
	  return boost::any_cast<Ty>(getUpper());
      }
           
      virtual std::string getStringLower() const;
      virtual std::string getStringUpper() const;
    protected:
      explicit BasicInterval(TREX::utils::Symbol const &type) 
	:DomainBase(type) {}
      explicit BasicInterval(rapidxml::xml_node<> const &node)
	:DomainBase(node) {}

      void completeParsing(rapidxml::xml_node<> const &node);

      virtual void parseLower(std::string const &val) =0;
      virtual void parseUpper(std::string const &val) =0;
      virtual std::ostream &print_lower(std::ostream &out) const =0;
      virtual std::ostream &print_upper(std::ostream &out) const =0;
      
    private:
      void accept(DomainVisitor &visitor) const;
      std::ostream &print_domain(std::ostream &out) const;
      boost::any singleton() const {
	return getLower();
      }
      std::string stringSingleton() const {
	return getStringLower();
      }
   }; // TREX::transaction::BasicInterval


  } // TREX::transaction
} // TREX 

#endif // H_BasicInterval
