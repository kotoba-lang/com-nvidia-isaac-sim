(ns genesis.articulation3d
  "Clean-room 3-D reduced-coordinate articulated rigid-body dynamics (the
  algorithm class NVIDIA PhysX uses for its `Articulation`): RNEA + CRBA +
  LDL^T over `genesis.spatial`'s 6-D spatial-vector algebra, for
  arbitrary 3-D joint axes (not just the planar single-axis case of
  `genesis.planar-chain`).

  Scoping decision (large file, 1564 lines in the original, ADR-2607010930
  scoped-restoration pattern): the *data model* — `JointType3d`, `Body3d`
  (per-body joint/inertia/limit parameters), `Articulation3dConfig`,
  `Articulation3dState` — is pure data with no `kami_articulated`
  dependency and is ported 1:1 below, along with the small pure helpers
  `movable?`/`zeros-state`/`n-bodies`.

  The numerical solver core (RNEA forward/inverse dynamics, CRBA mass
  matrix, forward-kinematics tree traversal, DLS position/pose IK, point
  and geometric Jacobians, energy) is a ~1400-line dense spatial-algebra
  implementation built directly on `genesis.spatial`'s primitives (already
  fully ported) — it is clean-room CPU math, not PhysX/OmniKit/wgpu native
  code, but its scale relative to the other 22 files puts a faithful 1:1
  port of the full solver out of scope for this restoration pass (see
  README). It is a documented gap, not a native-code exclusion: a
  follow-up restoration could port `step`/`inverse_dynamics`/`fk_world`/
  `mass_matrix`/`geometric_jacobian` directly atop `genesis.spatial`,
  which already carries the load-bearing Plucker-transform / spatial-
  inertia / cross-product primitives those functions compose.
  `from_articulated_system` (the URDF-driven constructor) additionally
  requires the not-restored `kami_articulated` crate and would remain
  excluded regardless.

  Restored from kotoba-lang/kami-engine `kami-genesis/src/articulation3d.rs`
  (deleted PR #82) as zero-dependency portable CLJC. Per ADR-2607010930."
  )

;; JointType3d
(def joint-type-fixed :fixed)
(def joint-type-revolute :revolute)
(def joint-type-prismatic :prismatic)

(defn ->body3d
  "One body + the joint connecting it to its parent. Topologically ordered
  (`parent` index < this body's own index; `parent` = -1 means base)."
  [{:keys [name parent joint-type axis e-tree r-tree inertia mass com
           lower upper has-limit? effort damping dof]}]
  {:name name :parent parent :joint-type joint-type :axis axis
   :e-tree e-tree :r-tree r-tree :inertia inertia :mass mass :com com
   :lower lower :upper upper :has-limit? has-limit? :effort effort
   :damping damping :dof dof})

(defn movable? [{:keys [joint-type]}] (not= joint-type joint-type-fixed))

(defn ->articulation3d-config [bodies gravity dt ndof]
  {:bodies bodies :gravity gravity :dt dt :ndof ndof})

(defn n-bodies [{:keys [bodies]}] (count bodies))

(defn zeros-state [ndof]
  {:q (vec (repeat ndof 0.0)) :qdot (vec (repeat ndof 0.0))})
