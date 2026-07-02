(ns genesis.isaac-api-test
  "Smoke tests for the config/delegation-only `genesis.isaac-api` scoped
  port (see its docstring for the scoping rationale)."
  (:require [clojure.test :refer [deftest is]]
            [genesis.isaac-api :as ia]))

(deftest clock-advances-and-resets
  (let [w0 (ia/new-isaac-world (/ 1.0 60.0))]
    (is (= (ia/current-time-step-index w0) 0))
    (is (< (Math/abs (- (ia/current-time w0) 0.0)) 1e-9))
    (let [w1 (nth (iterate ia/step w0) 5)]
      (is (= (ia/current-time-step-index w1) 5))
      (is (< (Math/abs (- (ia/current-time w1) (* 5 (/ 1.0 60.0)))) 1e-9))
      (let [w2 (ia/reset w1)]
        (is (= (ia/current-time-step-index w2) 0))))))
