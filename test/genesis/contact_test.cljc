(ns genesis.contact-test
  "Tests for the ported `genesis.contact` obstacle-contact geometry (the
  `Obstacle`/`Collider` shape data + plane/AABB/convex contact-generation
  math is fully ported; the coupled PGS multi-body solver is scoped out —
  see `genesis.contact` docstring)."
  (:require [clojure.test :refer [deftest is]]
            [genesis.contact :as contact]
            [genesis.convex :as convex]))

(deftest plane-contact-when-penetrating
  (let [plane (contact/obstacle-plane [0.0 0.0 1.0] 0.0)
        c (contact/obstacle-contact plane 0 [0.0 0.0 0.3] 0.5 1e-3)]
    (is (some? c))
    (is (= (:n c) [0.0 0.0 1.0]))
    (is (< (Math/abs (- (:depth c) 0.2)) 1e-9))))

(deftest plane-no-contact-when-clear
  (let [plane (contact/obstacle-plane [0.0 0.0 1.0] 0.0)]
    (is (nil? (contact/obstacle-contact plane 0 [0.0 0.0 5.0] 0.5 1e-3)))))

(deftest aabb-contact-outside-box
  (let [aabb (contact/obstacle-aabb [-1.0 -1.0 -1.0] [1.0 1.0 1.0])
        c (contact/obstacle-contact aabb 0 [0.0 0.0 1.3] 0.5 1e-3)]
    (is (some? c))
    (is (< (Math/abs (- (:depth c) 0.2)) 1e-9))))

(deftest aabb-contact-centre-inside-exits-nearest-face
  (let [aabb (contact/obstacle-aabb [-1.0 -1.0 -1.0] [1.0 1.0 1.0])
        c (contact/obstacle-contact aabb 0 [0.0 0.0 0.9] 0.1 1e-3)]
    (is (some? c))
    (is (= (:n c) [0.0 0.0 1.0]))))

(deftest convex-obstacle-contact-for-sphere-outside
  (let [poly (convex/box-at [0.0 0.0 0.0] [0.5 0.5 0.5])
        obstacle (contact/obstacle-convex poly)
        c (contact/obstacle-contact obstacle 0 [1.0 0.0 0.0] 0.6 1e-3)]
    (is (some? c))
    (is (> (Math/abs (nth (:n c) 0)) 0.9))))
