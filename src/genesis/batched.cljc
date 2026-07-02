(ns genesis.batched
  "Multi-environment articulation view (Isaac Sim tensor semantics):
  `ArticulationBatch` provides the `[num_envs, n_dof]` env-major flat
  tensor shape Isaac Lab expects, over `genesis.articulation3d`'s
  Spatial3d (Featherstone) solver.

  Scoping decision: `ArticulationBatch` (and its `PdDrive` implicit-PD/
  computed-torque actuator) is built directly on `Articulation3dConfig`/
  `Articulation3dState`, whose numerical solver core is scoped to
  config/constants-only in `genesis.articulation3d` (see that namespace's
  docstring for the full rationale). Since `ArticulationBatch::step` calls
  straight through to that solver for every env in the batch, this module
  is likewise ported as data schema only: the `ArticulationBatch`/`PdDrive`
  record shapes and the env-count/DOF-count bookkeeping (`num-envs`,
  `num-dof`, `dof-names`, `dof-limits` — all pure metadata, no dynamics)
  below are 1:1; the tensor step/actuator/IK/domain-randomization methods
  are left as a documented gap pending a follow-up `genesis.articulation3d`
  solver-core restoration.

  Restored from kotoba-lang/kami-engine `kami-genesis/src/batched.rs`
  (deleted PR #82) as zero-dependency portable CLJC. Per ADR-2607010930."
  )

(defn ->pd-drive
  [{:keys [targets vel-targets kps kds gravity-comp? computed-torque? accel-ff]}]
  {:targets targets :vel-targets vel-targets :kps kps :kds kds
   :gravity-comp? gravity-comp? :computed-torque? computed-torque? :accel-ff accel-ff})

(defn new-batch
  "`Articulation3dConfig` cloned across `num-envs` environments (all
  zeroed). Mirrors `ArticulationBatch::new` field layout; `:states`,
  `:efforts`, and `:last-efforts` are pre-sized but the dynamics step that
  would populate them over time is not ported (see namespace docstring)."
  [cfg num-envs]
  (let [num-envs (max num-envs 1)
        ndof (:ndof cfg)]
    {:cfg cfg
     :states (vec (repeat num-envs {:q (vec (repeat ndof 0.0)) :qdot (vec (repeat ndof 0.0))}))
     :efforts (vec (repeat (* num-envs ndof) 0.0))
     :pos-drive nil
     :dof-names []
     :dof-limits (vec (repeat ndof [##-Inf ##Inf]))
     :last-efforts (vec (repeat (* num-envs ndof) 0.0))
     :per-env-cfg nil}))

(defn num-envs [{:keys [states]}] (count states))
(defn num-dof [{:keys [cfg]}] (:ndof cfg))
(defn dof-names [{:keys [dof-names]}] dof-names)
(defn get-dof-index [batch name]
  (first (keep-indexed (fn [i n] (when (= n name) i)) (dof-names batch))))
(defn get-dof-limits [{:keys [dof-limits]}] dof-limits)
