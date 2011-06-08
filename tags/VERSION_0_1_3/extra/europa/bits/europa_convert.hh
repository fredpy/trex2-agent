#ifndef H_europa_convert
# define H_europa_convert 

# include <PLASMA/Domain.hh>
# include <PLASMA/DataType.hh>

# include <trex/domain/DomainVisitor.hh>

namespace TREX {
  namespace europa {
    namespace details {
      
      TREX::transaction::DomainBase *trex_domain(EUROPA::Domain const &dom);

      /** @brief Conversion to europa domain
       * 
       * This visitor class allow to convert a TREX domain to its europa 
       * framework representation
       * 
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup europa
       */
      class europa_domain :public TREX::transaction::DomainVisitor {
      public:
        /** @brief Constructor
         *
         * @param type A europa type descriptor
         * 
         * Create a new instance statring with a full domain of type @p type
         */
        explicit europa_domain(EUROPA::DataTypeId const &type) 
          :m_delete(true), m_dom(type->baseDomain().copy()), m_type(type) {}
        /** @brief Constructor 
         * 
         * @param dom A europa domain
         *
         * Create a new instance starting with a domain which is a copy of 
         * @p dom
         */
        explicit europa_domain(EUROPA::Domain const &dom) 
          :m_delete(true), m_dom(dom.copy()), m_type(dom.getDataType()) {}
        /** @brief Constructor 
         * 
         * @param dom A pointer to a europa domain
         *
         * Create a new instance referring to @p dom. This means that all the 
         * modification will be directly applied to @p dom  
         */
        explicit europa_domain(EUROPA::Domain *dom) 
          :m_delete(false), m_dom(dom), m_type(dom->getDataType()) {}
        ~europa_domain() {
          if( m_delete ) 
            delete m_dom;
        }
        
        void visit(TREX::transaction::BasicEnumerated const *dom);
        void visit(TREX::transaction::BasicInterval const *dom);
        void visit(TREX::transaction::DomainBase const *dom, bool);
        
        /** @brief Get domain 
         * 
         * Gets the current value of the europa domain
         *
         * @return the europa domain manipulated by this visitor
         * @{
         */
        EUROPA::Domain const &domain() const {
          return *m_dom;
        }
        EUROPA::Domain &domain() {
          return *m_dom;
        }
        /** @} */
      private:
        bool               m_delete;
        EUROPA::Domain    *m_dom;
        EUROPA::DataTypeId m_type;	
      }; // TREX::europa::details::europa_domain
      
      inline void europa_restrict(EUROPA::ConstrainedVariableId const &var,
                                  TREX::transaction::DomainBase const &dom) {
        europa_domain convert(var->lastDomain());
        dom.accept(convert);
        var->restrictBaseDomain(convert.domain());
      }
      
    } // TREX::europa::details
  } // TREX::europa
} // TREX

#endif // H_EUROPA_CONVERT
