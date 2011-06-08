/* -*- C++ -*- */
/** @file "DomainVisitor.hh"
 * @brief Definition of DomainVisitor
 *
 * This file defines the abstract interface DomainVisitor used to visit
 * TREX domains
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef H_DomainVisitor
# define H_DomainVisitor

# include "BasicEnumerated.hh"
# include "BasicInterval.hh"

namespace TREX {
  namespace transaction {

    /** @brief abstract domain visitor interface
     *
     * This class defines the abstract interface used to implement a visitor
     * of TREX domains (aka DomainBase)
     *
     * It allows to visit different standard types of domain and embeds a
     * default behavior for classes that do  not inherit from these.
     *
     * @sa void DomainBase::accept(DomainVisitor &)
     * @author Frederic Py <fpy@mbari.org>
     */
    class DomainVisitor {
    public:
      /** @brief Enumerated domain visit
       *
       * @param dom A domain
       *
       * This method is called when visiting the BasicEnumerated domain @e dom
       */
      virtual void visit(BasicEnumerated const *dom) = 0;
      /** @brief Interval domain visit
       *
       * @param dom A domain
       *
       * This method is called when visiting the BasicInterval domain @e dom
       */
      virtual void visit(BasicInterval const *dom) =0;
      /** @brief Default visit
       *
       * @param dom A domain
       *
       * This method is called when visiting a domain @e dom which is not a
       * BasicInterval nor a BasicEnumerated class
       */
      virtual void visit(DomainBase const *dom, bool) =0;
    };

  }
}

#endif // H_DomainVisitor
