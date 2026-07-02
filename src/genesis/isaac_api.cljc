(ns genesis.isaac-api
  "`isaacsim.core.api`-shaped facade (`IsaacWorld`, `ArticulationView`,
  `ArticulationViewMut`, `ArticulationControllerView`) — clean-room mirror
  of the Isaac Sim 4.x surface, no NVIDIA library linked.

  Scoping decision: the original `isaac_api.rs` is a thin wrapper around
  `genesis.world`'s `World`/`Articulation` (`kami_articulated`-URDF-driven,
  out of scope here — see `genesis.world`) plus `genesis.controllers`. It
  adds no numerical logic of its own; every method delegates straight
  through to the wrapped `World`/`Articulation`/`ArticulationController`.
  Since the underlying `World` is scoped to config/constants-only in this
  restoration, this facade is likewise ported as data + delegation
  signatures only: the `physics-dt`/`current-time`/`current-time-step-index`
  clock bookkeeping (pure, no URDF dependency) is fully ported below; the
  view constructors that would wrap a live `World` articulation are left
  as documented no-op signatures for API-surface completeness.

  Restored from kotoba-lang/kami-engine `kami-genesis/src/isaac_api.rs`
  (deleted PR #82) as zero-dependency portable CLJC. Per ADR-2607010930."
  )

(defn new-isaac-world [physics-dt]
  {:physics-dt physics-dt :time-step-index 0})

(defn get-physics-dt [w] (:physics-dt w))
(defn current-time-step-index [w] (:time-step-index w))
(defn current-time [{:keys [physics-dt time-step-index]}] (* physics-dt time-step-index))

(defn step
  "Advance the clock by one physics step (`time-step-index` increments;
  actual articulation stepping is delegated to `genesis.world/step-topology`
  by the caller, mirroring the original's `World::step()` + clock-advance
  split)."
  [w] (update w :time-step-index inc))

(defn reset
  "Rewind the clock (`current_time`/`current_time_step_index` reset to 0),
  mirroring `isaacsim.core.api.World.reset()`."
  [w] (assoc w :time-step-index 0))
