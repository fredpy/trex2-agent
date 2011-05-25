/** @file "Observation.cc"
 * @brief Observation implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */
# include "Observation.hh"

using namespace TREX::utils;
using namespace TREX::transaction;

namespace {

  /** @brief Observation declaration for XML parsing
   * @ingroup transaction
   */
  Predicate::xml_factory::declare<Observation> decl_obs("Observation");

}

Symbol const &Observation::getPredTag() const {
  static Symbol const name("Observation");
  return name;
}
