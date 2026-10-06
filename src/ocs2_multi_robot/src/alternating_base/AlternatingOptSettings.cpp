#include "ocs2_multi_robot/alternating_base/AlternatingOptSettings.h"

#include <algorithm>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace multi_robot {

AlternatingOptSettings loadAlternatingSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  if (verbose) {
    std::cerr << std::endl << " #### ADMM Settings: " << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }
  AlternatingOptSettings settings;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  const std::string prefix = fieldName + ".";

  loadData::loadPtreeValue(pt, settings.max_iter, prefix + "max_iter", verbose);
  loadData::loadPtreeValue(pt, settings.printSolverStatus, prefix + "printSolverStatus", verbose);
  loadData::loadPtreeValue(pt, settings.printSolverStatistics, prefix + "printSolverStatistics", verbose);
  loadData::loadPtreeValue(pt, settings.numConsensusConstraints, prefix + "numConsensusConstraints", verbose);
  loadData::loadPtreeValue(pt, settings.primalTolerance, prefix + "primalTolerance", verbose);
  loadData::loadPtreeValue(pt, settings.parallelUpdate, prefix + "parallelUpdate", verbose);
  loadData::loadPtreeValue(pt, settings.enableAdaptiveRho, prefix + "enableAdaptiveRho", verbose);
  loadData::loadPtreeValue(pt, settings.adaptiveRhoMu, prefix + "adaptiveRhoMu", verbose);
  loadData::loadPtreeValue(pt, settings.adaptiveRhoTauIncr, prefix + "adaptiveRhoTauIncr", verbose);
  loadData::loadPtreeValue(pt, settings.adaptiveRhoTauDecr, prefix + "adaptiveRhoTauDecr", verbose);
  loadData::loadPtreeValue(pt, settings.resetDualWhenSingleIteration, prefix + "resetDualWhenSingleIteration", verbose);
  loadData::loadPtreeValue(pt, settings.dualResetCostThreshold, prefix + "dualResetCostThreshold", verbose);
  loadData::loadPtreeValue(pt, settings.logMpcResiduals, prefix + "logMpcResiduals", verbose);
  loadData::loadPtreeValue(pt, settings.mpcResidualLogPath, prefix + "mpcResidualLogPath", verbose);
  std::string updateTypeName;
  loadData::loadPtreeValue(pt, updateTypeName, prefix + "updateType", verbose);
  settings.updateType = fromString(updateTypeName);
  if (verbose) {
    std::cerr << "[AlternatingOptSettings] Update type: " << updateTypeName << std::endl;
  }

  settings.rho = vector_t::Zero(settings.numConsensusConstraints);
  try {
    loadData::loadEigenMatrix(filename, prefix + "rho", settings.rho);
  } catch (const std::exception& e) {
    if (verbose) {
      std::cerr << "[AlternatingOptSettings] Using default rho. Reason: " << e.what() << '\n';
    }
  }
  if (settings.numConsensusConstraints <= 0) {
    settings.numConsensusConstraints = settings.rho.size();
  }
  if (settings.rho.size() == 0) {
    settings.rho = vector_t::Ones(std::max(1, settings.numConsensusConstraints));
  } else if (settings.rho.size() != settings.numConsensusConstraints && settings.numConsensusConstraints > 0) {
    if (verbose) {
      std::cerr << "[AlternatingOptSettings] Resizing rho from " << settings.rho.size() << " to "
                << settings.numConsensusConstraints << " using the first value.\n";
    }
    const scalar_t value = settings.rho.size() > 0 ? settings.rho(0) : 1.0;
    settings.rho = vector_t::Constant(settings.numConsensusConstraints, value);
  }

  return settings;
}

}  // namespace multi_robot
}  // namespace ocs2


