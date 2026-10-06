#pragma once

#include <memory>
#include <mutex>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>

namespace ocs2 {
namespace multi_robot {

/**
 * Thread-safe wrapper for ReferenceManager.
 *
 * In addition to protecting preSolverRun() calls, this also serializes access to
 * getModeSchedule(), getTargetTrajectories(), and their corresponding setters.
 * This is important when multiple SQP solvers are run in parallel (one per
 * robot and cargo) and all share the same underlying ReferenceManager.
 */
class ThreadSafeReferenceManager : public ReferenceManagerDecorator {
 public:
  explicit ThreadSafeReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
      : ReferenceManagerDecorator(std::move(referenceManagerPtr)) {}

  ~ThreadSafeReferenceManager() override = default;

  /**
   * Protects preSolverRun(), which may update internal buffered references.
   *
   * Additionally, ensures that the underlying ReferenceManager::preSolverRun()
   * is executed at most once per MPC horizon (initTime, finalTime). Subsequent
   * calls with the same horizon will be no-ops, which avoids redundant
   * reference updates when multiple subproblems share this manager.
   */
  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const scalar_t tol = 1e-9;
    const bool sameInit  = horizonInitialized_ && std::abs(initTime  - lastInitTime_)  <= tol;
    const bool sameFinal = horizonInitialized_ && std::abs(finalTime - lastFinalTime_) <= tol;

    // Only propagate to the wrapped manager if this is a new horizon
    if (!sameInit || !sameFinal) {
      referenceManagerPtr_->preSolverRun(initTime, finalTime, initState);
      lastInitTime_ = initTime;
      lastFinalTime_ = finalTime;
      horizonInitialized_ = true;
    }
  }

  /** Thread-safe access to the active ModeSchedule. */
  const ModeSchedule& getModeSchedule() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return referenceManagerPtr_->getModeSchedule();
  }

  /** Thread-safe buffered update of ModeSchedule. */
  void setModeSchedule(const ModeSchedule& modeSchedule) override {
    std::lock_guard<std::mutex> lock(mutex_);
    referenceManagerPtr_->setModeSchedule(modeSchedule);
  }

  /** Thread-safe buffered update of ModeSchedule (rvalue overload). */
  void setModeSchedule(ModeSchedule&& modeSchedule) override {
    std::lock_guard<std::mutex> lock(mutex_);
    referenceManagerPtr_->setModeSchedule(std::move(modeSchedule));
  }

  /** Thread-safe access to the active TargetTrajectories. */
  const TargetTrajectories& getTargetTrajectories() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return referenceManagerPtr_->getTargetTrajectories();
  }

  /** Thread-safe buffered update of TargetTrajectories. */
  void setTargetTrajectories(const TargetTrajectories& targetTrajectories) override {
    std::lock_guard<std::mutex> lock(mutex_);
    referenceManagerPtr_->setTargetTrajectories(targetTrajectories);
  }

  /** Thread-safe buffered update of TargetTrajectories (rvalue overload). */
  void setTargetTrajectories(TargetTrajectories&& targetTrajectories) override {
    std::lock_guard<std::mutex> lock(mutex_);
    referenceManagerPtr_->setTargetTrajectories(std::move(targetTrajectories));
  }

 private:
  // Single mutex guarding all access to the wrapped ReferenceManager. This avoids
  // concurrent reads/writes of the underlying ModeSchedule / TargetTrajectories
  // when multiple SQP solvers are run in parallel by the ADMM solver.
  mutable std::mutex mutex_;

  // Cache of the last MPC horizon for which preSolverRun() has been executed.
  scalar_t lastInitTime_{0.0};
  scalar_t lastFinalTime_{0.0};
  bool horizonInitialized_{false};
};

}  // namespace multi_robot
}  // namespace ocs2

