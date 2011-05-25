#ifndef H_EuropaDomain 
# define H_EuropaDomain

# include <trex/domain/EnumeratedDomain.hh>

# include <PLASMA/Domain.hh>

namespace TREX {
  namespace europa {
    
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

    class EuropaEntity :public TREX::transaction::EnumeratedDomain<TREX::utils::Symbol> {
      typedef TREX::transaction::EnumeratedDomain<TREX::utils::Symbol> base_class;
    public:
      static TREX::utils::Symbol const type_name;
      
      EuropaEntity()
	:base_class(type_name) {}
      template<class Iter>
      EuropaEntity(Iter from , Iter to) 
	:base_class(type_name, from, to) {}
      explicit EuropaEntity(TREX::utils::Symbol const &val) 
	:base_class(type_name, val) {}
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
