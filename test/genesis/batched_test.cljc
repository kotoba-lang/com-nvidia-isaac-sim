(ns genesis.batched-test
  "Smoke tests for the config/data-only `genesis.batched` scoped port
  (see its docstring for the scoping rationale)."
  (:require [clojure.test :refer [deftest is]]
            [genesis.batched :as batched]
            [genesis.articulation3d :as a3d]))

(deftest batch-shapes-match-num-envs-and-ndof
  (let [cfg (a3d/->articulation3d-config [] [0.0 0.0 -9.81] 0.01 3)
        b (batched/new-batch cfg 8)]
    (is (= (batched/num-envs b) 8))
    (is (= (batched/num-dof b) 3))
    (is (= (count (:efforts b)) 24))
    (is (= (count (batched/get-dof-limits b)) 3))))

(deftest batch-clamps-num-envs-to-at-least-1
  (let [cfg (a3d/->articulation3d-config [] [0.0 0.0 -9.81] 0.01 1)
        b (batched/new-batch cfg 0)]
    (is (= (batched/num-envs b) 1))))

(deftest get-dof-index-looks-up-name
  (let [cfg (a3d/->articulation3d-config [] [0.0 0.0 -9.81] 0.01 2)
        b (assoc (batched/new-batch cfg 1) :dof-names ["shoulder" "elbow"])]
    (is (= (batched/get-dof-index b "elbow") 1))
    (is (nil? (batched/get-dof-index b "wrist")))))
