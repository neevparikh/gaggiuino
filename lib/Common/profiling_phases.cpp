/* 09:32 15/03/2023 - change triggering comment */
#include "profiling_phases.h"

//----------------------------------------------------------------------//
//------------------------- ShotSnapshot -------------------------------//
//----------------------------------------------------------------------//

ShotSnapshot buildShotSnapshot(uint32_t timeInShot, const SensorState& state, CurrentPhase& phase) {
  float targetFlow = (phase.getType() == PHASE_TYPE::PHASE_TYPE_FLOW) ? phase.getTarget() : phase.getRestriction();
  float targetPressure = (phase.getType() == PHASE_TYPE::PHASE_TYPE_PRESSURE) ? phase.getTarget() : phase.getRestriction();

  return ShotSnapshot{
    .timeInShot=timeInShot,
    .pressure=state.smoothedPressure,
    .pumpFlow=state.smoothedPumpFlow,
    .weightFlow=state.smoothedWeightFlow,
    .temperature=state.temperature,
    .shotWeight=state.shotWeight,
    .waterPumped=state.waterPumped,
    .targetTemperature=-1,
    .targetPumpFlow=targetFlow,
    .targetPressure=targetPressure
  };
};

//----------------------------------------------------------------------//
//------------------------------ Phase ---------------------------------//
//----------------------------------------------------------------------//
float Phase::getTarget(uint32_t timeInPhase, const ShotSnapshot& stateAtStart) const {
  long transitionTime = fmax(0L, target.time);
  float startValue = target.start > 0.f
    ? target.start
    : type == PHASE_TYPE::PHASE_TYPE_FLOW ? stateAtStart.pumpFlow : stateAtStart.pressure;

  return mapRange(timeInPhase, 0.f, transitionTime, startValue, target.end, 1, target.curve);
}

float Phase::getRestriction() const {
  return restriction;
}

bool Phase::isStopConditionReached(SensorState& currentState, const eepromValues_t currentConfig, uint32_t timeInShot, ShotSnapshot stateAtPhaseStart) const {
  return stopConditions.isReached(currentState, currentConfig, timeInShot, stateAtPhaseStart);
}

//----------------------------------------------------------------------//
//-------------------------- StopConditions ----------------------------//
//----------------------------------------------------------------------//
inline bool predictShotCompletion(const float targetDose, const float currentDose, const float flowRate) {
  float remainingDose = targetDose - currentDose;
  float percentRemaining = remainingDose / flowRate;

  return percentRemaining < 0.3f ? true : false;
}

bool PhaseStopConditions::isReached(SensorState& state, const eepromValues_t currentConfig, long timeInShot, ShotSnapshot stateAtPhaseStart) const {
  float desiredShotWeight = -1.f;
  bool stopOnWeightReached = false;
  float flow = state.weight > 0.4f ? state.smoothedWeightFlow : state.smoothedPumpFlow;
  float stopDelta = flow * state.shotWeight / 100.f;
  if (ACTIVE_PROFILE(currentConfig).stopOnWeightState) {
    desiredShotWeight = ACTIVE_PROFILE(currentConfig).shotStopOnCustomWeight > 1.f
                        ? ACTIVE_PROFILE(currentConfig).shotStopOnCustomWeight
                        : ACTIVE_PROFILE(currentConfig).shotDose * ACTIVE_PROFILE(currentConfig).shotPreset;
    stopOnWeightReached = predictShotCompletion(desiredShotWeight, state.shotWeight, flow);
  }

  return (time >= 0L && timeInShot - stateAtPhaseStart.timeInShot >= (uint32_t) time) ||
    (weight > 0.f && stopOnWeightReached) ||
    (weight > 0.f && state.shotWeight > weight ) ||
    (pressureAbove > 0.f && state.smoothedPressure > pressureAbove) ||
    (pressureBelow > 0.f && state.smoothedPressure < pressureBelow) ||
    (waterPumpedInPhase > 0.f &&  state.waterPumped - stateAtPhaseStart.waterPumped > waterPumpedInPhase - stopDelta) ||
    (flowAbove > 0.f && state.smoothedPumpFlow > flowAbove) ||
    (flowBelow > 0.f && state.smoothedPumpFlow < flowBelow);
}

bool GlobalStopConditions::isReached(const SensorState& state, const eepromValues_t currentConfig, long timeInShot) {
  float desiredShotWeight = -1.f;
  bool stopOnWeightReached = false;
  float flow = state.weight > 0.4f ? state.smoothedWeightFlow : state.smoothedPumpFlow;
  float stopDelta = flow * (state.shotWeight / 100.f);
  if (ACTIVE_PROFILE(currentConfig).stopOnWeightState) {
    desiredShotWeight = ACTIVE_PROFILE(currentConfig).shotStopOnCustomWeight > 1.f
                        ? ACTIVE_PROFILE(currentConfig).shotStopOnCustomWeight
                        : ACTIVE_PROFILE(currentConfig).shotDose * ACTIVE_PROFILE(currentConfig).shotPreset;
    stopOnWeightReached = predictShotCompletion(desiredShotWeight, state.shotWeight, flow);
  }

  return (weight > 0.f && stopOnWeightReached) ||
    (waterPumped > 0.f && state.waterPumped > waterPumped) ||
    (time >= 0L && timeInShot >= time);
}

//----------------------------------------------------------------------//
//--------------------------- CurrentPhase -----------------------------//
//----------------------------------------------------------------------//
CurrentPhase::CurrentPhase(int index, const Phase& phase, uint32_t timeInPhase, const ShotSnapshot& shotSnapshotAtStart) : index(index), phase{ &phase }, timeInPhase(timeInPhase), shotSnapshotAtStart{ &shotSnapshotAtStart} {}
CurrentPhase::CurrentPhase(const CurrentPhase& currentPhase) : index(currentPhase.index), phase{ currentPhase.phase }, timeInPhase(currentPhase.timeInPhase), shotSnapshotAtStart{ currentPhase.shotSnapshotAtStart} {}

Phase CurrentPhase::getPhase() { return *phase; }

PHASE_TYPE CurrentPhase::getType() { return phase->type; }

int CurrentPhase::getIndex() { return index; }

long CurrentPhase::getTimeInPhase() { return timeInPhase; }

float CurrentPhase::getTarget() { return phase->getTarget(timeInPhase, *shotSnapshotAtStart); }

float CurrentPhase::getRestriction() { return phase->getRestriction(); }

void CurrentPhase::update(int index, Phase& phase, uint32_t timeInPhase) {
  CurrentPhase::index = index;
  CurrentPhase::phase = &phase;
  CurrentPhase::timeInPhase = timeInPhase;
}

//----------------------------------------------------------------------//
//-------------------------- PhaseProfiler -----------------------------//
//----------------------------------------------------------------------//

PhaseProfiler::PhaseProfiler(Profile& profile) : profile(profile) {}

void PhaseProfiler::updatePhase(uint32_t timeInShot, SensorState& state, const eepromValues_t currentConfig) {
  size_t phaseIdx = currentPhaseIdx;
  uint32_t timeInPhase = timeInShot - phaseChangedSnapshot.timeInShot;

  if (phaseIdx >= profile.phaseCount() || profile.globalStopConditions.isReached(state, currentConfig, timeInShot)) {
    currentPhaseIdx = profile.phaseCount();
    phaseIdx = profile.phaseCount() - 1;
    currentPhase.update(phaseIdx, profile.phases[phaseIdx], timeInPhase);
    return;
  }

  if (!profile.phases[phaseIdx].isStopConditionReached(state, currentConfig, timeInShot, phaseChangedSnapshot)) {
    currentPhase.update(phaseIdx, profile.phases[phaseIdx], timeInPhase);
    return;
  }

  currentPhase.update(phaseIdx, profile.phases[phaseIdx], timeInPhase);
  phaseChangedSnapshot = buildShotSnapshot(timeInShot, state, currentPhase);
  currentPhaseIdx += 1;
  updatePhase(timeInShot, state, currentConfig);
}

// Gets the profiling phase we should be in based on the timeInShot and the Sensors state
CurrentPhase& PhaseProfiler::getCurrentPhase() {
  return currentPhase;
}

bool PhaseProfiler::isFinished() {
  return currentPhaseIdx >= profile.phaseCount();
}

void PhaseProfiler::reset() {
  currentPhaseIdx = 0;
  phaseChangedSnapshot = ShotSnapshot{};
  currentPhase.update(0, profile.phases[0], 0);
}
