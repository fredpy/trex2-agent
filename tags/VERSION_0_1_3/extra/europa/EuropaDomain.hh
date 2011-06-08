#ifndef H_EuropaDomain 
# define H_EuropaDomain

# include <trex/domain/EnumeratedDomain.hh>

# include <PLASMA/Domain.hh>

namespace TREX {
  namespace europa {
    
    /** @brief Proxy between trex and europa domains
     *
     * This class is a simpel proxy between europa domain internal representation
     * and TREX domain representation
     * 
     * It allows to easily access to europa domain information as if it were a 
     * TREX domain
     * 
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class EuropaDomain :public TREX::transaction::DomainBase {
    public:
      static TREX::utils::Symbol const type_name;
      
      explicit EuropaDomain(EUROPA::Domain const &dom);
      EuropaDomain(EuropaDomain const &other);
      ~EuropaDomain();
      
      TREX::transaction::DomainBase *copy() const;
      std::ostream &toXml(std::ostream &out, size_t tabs) const;
      
      bool isInterval() const {
        return m_dom->isInterval();
      }
      bool isEnumerated() const {
        return m_dom->isEnumerated();
      }
      bool isFull() const;	
      bool isSingleton() const {
        return m_dom->isSingleton();
      }
      bool intersect(TREX::transaction::DomainBase const &other) const;
      bool equals(TREX::transaction::DomainBase const &other) const;
      
      TREX::transaction::DomainBase &restrictWith(TREX::transaction::DomainBase const &other);
      
      EUROPA::Domain const &europaDomain() const {
        return *m_dom;
      }
      
    private:
      EUROPA::Domain *m_dom;
      
      static EUROPA::Domain *safe_copy(EUROPA::Domain *dom);
      
      std::ostream &print_domain(std::ostream &out) const;
      boost::any singleton() const;
      std::string stringSingleton() const; 
      
    }; // TREX::europa::EuropaDomain
    
    /** @brief Domain for europa objects
     * 
     * This domain is used to represent europa objects in TREX
     * 
     * @ingroup europa
     * @author Frederic Py <fpy@mbari.org>
     */
    class EuropaEntity :public TREX::transaction::EnumeratedDomain<TREX::utils::Symbol> {
      typedef TREX::transaction::EnumeratedDomain<TREX::utils::Symbol> base_class;
    public:
      /** @brief Type name 
       * 
       * The type used to identify  a domain of this type. This type is also 
       * used for serilaization of this class in XML form
       */
      static TREX::utils::Symbol const type_name;
      
      EuropaEntity()
        :base_class(type_name) {}
      /** @brief Constructor
       * 
       * @tparam Iter An iterator type
       * @param[in] from first iterator
       * @param[in] to last iterator
       * 
       * Create a new domain with all the elements between @p from and @p to
       * @pre [from, to) should be a valid iterator sequence
       * @pre [from to) is not empty (ie from!=to)
       * @pre the type referred by @e Iter should be convertible into 
       *      TREX::utils::Symbol
       * @throw EmptyDomain the created domain is empty
       */
      template<class Iter>
      EuropaEntity(Iter from , Iter to) 
        :base_class(type_name, from, to) {}
      /** @brief Singleont instance
       *
       * @param[in] val A value
       *
       * Create new domain with the dingle value @p val
       */
      explicit EuropaEntity(TREX::utils::Symbol const &val) 
        :base_class(type_name, val) {}
      /** @brief XML parsing constructor
       * @param[in] node An XML node
       *
       * Create a new instance by parsing the content of @e node.
       * The expected structure of this node is expected to be of
       * a from similar to the following :
       * @code
       * <europa_object>
       *   <elem value="<val1>"/>
       *   <elem value="<val2>"/>
       *   [...]
       * </europa_object>
       * @endcode
       * and @c @<vali@> are possible values for this domain
       * @throw TREX::utils::bad_string_cast Unable to parse one of
       * the @c @<vali@> attributes
       *
       * @note If no @c elem tag is defined into @e node, the domain
       * will be considered as full.
       */
     explicit EuropaEntity(rapidxml::xml_node<> const &node)
        :base_class(node) {}
      ~EuropaEntity() {}
      
      TREX::transaction::DomainBase *copy() const {
        return new EuropaEntity(*this);
      }
    }; // TREX::europa::EuropaEntity
    
  } // TREX::europa
} // TREX

#endif // H_EuropaDomain
