(ns genesis.world
  "`World` + `Articulation` — Isaac Sim/PhysX-style API surface.

  Scoping decision (native-facade-dependent file, ADR-2607010930
  scoped-restoration pattern): the original `World`/`Articulation` are
  built via `Articulation::from_urdf`, which requires
  `kami_articulated::{ArticulatedSystem, JointKind}` (a separate,
  not-restored URDF-parsing crate) to turn a URDF file into a rigid-body
  topology. Every other module in this restoration that needs a `World`
  (`genesis.controllers`, `genesis.trajectory`'s integration test) is
  adapted instead to operate on the closed-form topology states directly
  (`genesis.cartpole`/`genesis.double-pendulum`/`genesis.planar-chain`),
  which is a faithful port of the *dynamics*, just not of this
  URDF-driven container.

  What IS ported here (pure data, no `kami_articulated` dependency): the
  `LinkState` per-link kinematic pose/twist record, the default `World`
  scene parameters (gravity, dt), and the `WorldError` variants — plus a
  small multi-topology dispatcher `step-topology`/`jacobian-for-link` that
  mirrors `Articulation::step`/`Articulation::jacobian` for the three
  closed-form topologies this crate implements natively (Cartpole,
  DoublePendulum, PlanarChain), operating on a `{:topology :cfg :state}`
  map instead of a `kami_articulated`-parsed generic articulation.

  Restored from kotoba-lang/kami-engine `kami-genesis/src/world.rs`
  (deleted PR #82) as zero-dependency portable CLJC. Per ADR-2607010930."
  (:require [genesis.cartpole :as cartpole]
            [genesis.double-pendulum :as dp]
            [genesis.planar-chain :as pc]
            [genesis.jacobian :as jac]))

(defn link-state-at-origin []
  {:position [0.0 0.0 0.0] :orientation [0.0 0.0 0.0 1.0] ;; quat wxyz->xyzw irrelevant at identity
   :linear-velocity [0.0 0.0 0.0] :angular-velocity [0.0 0.0 0.0]})

(defn default-world []
  {:gravity 9.81 :dt (/ 1.0 60.0) :articulations []})

;; WorldError variants (as keyword + message, mirroring the Rust `thiserror`
;; messages 1:1):
(defn unsupported-topology-error [name]
  {:kind :unsupported-topology
   :message (str "articulation topology not supported at R1.1: " name
                 ". Cartpole (1 prismatic + 1 revolute) is the only supported topology.")})
(defn invalid-handle-error [h] {:kind :invalid-handle :message (str "articulation handle " h " is invalid")})
(defn duplicate-name-error [name] {:kind :duplicate-name :message (str "articulation `" name "` already registered")})

;; ── Small multi-topology dispatcher over the closed-form solvers ────────

(defn step-topology
  "Steps a `{:topology (:cartpole|:double-pendulum|:planar-chain) :cfg
  :state}` map by one physics step under `action` (a scalar force for
  :cartpole, or a per-joint torque vector for the other two)."
  [{:keys [topology cfg state] :as art} action]
  (assoc art :state
    (case topology
      :cartpole (cartpole/step state action cfg)
      :double-pendulum (dp/step state action cfg)
      :planar-chain (pc/step state action cfg))))

(defn jacobian-for-link
  "Analytical Jacobian for `link` on a `{:topology :cfg :state}` map."
  [{:keys [topology cfg state]} link]
  (case topology
    :cartpole (jac/cartpole-link-jacobian (:theta state) link cfg)
    :double-pendulum (jac/dp-link-jacobian (:q1 state) (:q2 state) link cfg)
    :planar-chain (jac/planar-chain-link-jacobian (:q state) link cfg)))
