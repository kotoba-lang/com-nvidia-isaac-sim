(ns genesis.articulation3d-test
  "Smoke tests for the config/data-only `genesis.articulation3d` scoped
  port (see its docstring for the scoping rationale)."
  (:require [clojure.test :refer [deftest is]]
            [genesis.articulation3d :as a3d]
            [genesis.spatial :as sp]))

(deftest fixed-body-is-not-movable
  (let [b (a3d/->body3d {:name "base" :parent -1 :joint-type a3d/joint-type-fixed
                          :axis [0.0 0.0 1.0] :e-tree sp/mat3-identity :r-tree [0.0 0.0 0.0]
                          :inertia sp/zero-m6 :mass 0.0 :com [0.0 0.0 0.0]
                          :lower 0.0 :upper 0.0 :has-limit? false :effort 0.0 :damping 0.0 :dof -1})]
    (is (not (a3d/movable? b)))))

(deftest revolute-body-is-movable
  (let [b (a3d/->body3d {:name "link1" :parent 0 :joint-type a3d/joint-type-revolute
                          :axis [0.0 1.0 0.0] :e-tree sp/mat3-identity :r-tree [0.0 0.0 1.0]
                          :inertia sp/zero-m6 :mass 1.0 :com [0.0 0.0 0.5]
                          :lower -3.14 :upper 3.14 :has-limit? true :effort 50.0 :damping 0.1 :dof 0})]
    (is (a3d/movable? b))))

(deftest n-bodies-and-zeros-state
  (let [bodies [{:name "a"} {:name "b"}]
        cfg (a3d/->articulation3d-config bodies [0.0 0.0 -9.81] 0.01 2)]
    (is (= (a3d/n-bodies cfg) 2))
    (is (= (a3d/zeros-state 2) {:q [0.0 0.0] :qdot [0.0 0.0]}))))
