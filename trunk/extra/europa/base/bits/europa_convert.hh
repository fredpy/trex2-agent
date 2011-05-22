#ifndef H_europa_convert
# define H_europa_convert 

# include "PLASMA/Domain.hh"
# include "PLASMA/DataType.hh"

# include "DomainVisitor.hh"

namespace TREX {
  namespace europa {
    namespace details {
      
      TREX::transaction::DomainBase *trex_domain(EUROPA::Domain const &dom);
      
      class europa_domain :public TREX::transaction::DomainVisitor {
      public:
	explicit europa_domain(EUROPA::DataTypeId const &type) 
	  :m_delete(true), m_dom(type->baseDomain().copy()), m_type(type) {}
	explicit europa_domain(EUROPA::Domain const &dom) 
	  :m_delete(true), m_dom(dom.copy()), m_type(dom.getDataType()) {}
	explicit europa_domain(EUROPA::Domain *dom) 
	  :m_delete(false), m_dom(dom), m_type(dom->getDataType()) {}
	~europa_domain() {
	  if( m_delete ) 
	    delete m_dom;
	}
	
	void visit(TREX::transaction::BasicEnumerated const *dom);
	void visit(TREX::transaction::BasicInterval const *dom);
	void visit(TREX::transaction::DomainBase const *dom, bool);

	EUROPA::Domain const &domain() const {
	  return *m_dom;
	}
	EUROPA::Domain &domain() {
	  return *m_dom;
	}
      private:
	bool               m_delete;
	EUROPA::Domain    *m_dom;
	EUROPA::DataTypeId m_type;	
      }; // TREX::europa::details::europa_domain
      
    } // TREX::europa::details
  } // TREX::europa
} // TREX

#endif // H_EUROPA_CONVERT
