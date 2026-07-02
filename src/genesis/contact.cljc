(ns genesis.contact
  "Rigid contact / collision solver coupled to the 3-D reduced-coordinate
  articulation.

  Scoping decision (large/mixed file, ADR-2607010930 scoped-restoration
  pattern): the `Obstacle`/`Collider` shape data and the per-obstacle
  contact-generation geometry (`obstacle-contact` — plane / AABB / convex
  sphere-contact queries against a world-space sphere) are pure math over
  `genesis.vec3`/`genesis.convex` and are ported 1:1 below. `ContactWorld`
  itself — the velocity-level sequential-impulse / projected Gauss-Seidel
  solver that couples contacts into the articulation's *joint space* via
  the CRBA joint-space inertia (`Articulation3dConfig::point_jacobian` /
  `solve_ldlt`) — is NOT ported: it depends on the full
  `genesis.articulation3d` numerical solver core, which this restoration
  scopes to config/constants-only (see `genesis.articulation3d`), and on
  `kami_articulated`-parsed URDF systems for its test fixtures. Porting a
  correct multi-body PGS contact solver without that foundation would not
  be a faithful 1:1 port, so it is excluded here per the \"lighter-weight
  config/constants-only\" scoping guidance.

  Restored from kotoba-lang/kami-engine `kami-genesis/src/contact.rs`
  (deleted PR #82) as zero-dependency portable CLJC. Per ADR-2607010930."
  (:require [genesis.vec3 :as v3]
            [genesis.convex :as convex]))

;; ── Collider shapes (body-frame) ────────────────────────────────────────

(defn collider-sphere [center radius] {:kind :sphere :center center :radius radius})
(defn collider-capsule [a b radius] {:kind :capsule :a a :b b :radius radius})
(defn collider-box [center half radius] {:kind :box :center center :half half :radius radius})

;; ── Static environment obstacles ────────────────────────────────────────

(defn obstacle-plane [normal offset] {:kind :plane :normal normal :offset offset})
(defn obstacle-aabb [mn mx] {:kind :aabb :min mn :max mx})
(defn obstacle-convex [poly] {:kind :convex :poly poly})

(def default-contact-params
  {:ground-z 0.0 :restitution 0.0 :friction 0.8 :baumgarte 0.15 :slop 1.0e-3 :iters 12})

(defn- clamp3 [[x y z] [mnx mny mnz] [mxx mxy mxz]]
  [(max mnx (min mxx x)) (max mny (min mxy y)) (max mnz (min mxz z))])

(defn obstacle-contact
  "Contact for a world-space sphere `(c, radius)` belonging to `link`, or
  nil if it is clear of `obstacle` (beyond the slop band)."
  [obstacle link c radius slop]
  (case (:kind obstacle)
    :plane
    (let [n (v3/normalize (:normal obstacle))
          d (- (v3/dot c n) (:offset obstacle))
          depth (- radius d)]
      (when (> depth (- slop)) {:link link :p (v3/sub c (v3/scale n d)) :n n :depth depth}))

    :aabb
    (let [{mn :min mx :max} obstacle
          cp (clamp3 c mn mx)
          diff (v3/sub c cp)
          dist2 (v3/length-squared diff)]
      (if (> dist2 1.0e-12)
        (let [dist (Math/sqrt dist2) n (v3/scale diff (/ 1.0 dist)) depth (- radius dist)]
          (when (> depth (- slop)) {:link link :p cp :n n :depth depth}))
        (let [dlo (v3/sub c mn) dhi (v3/sub mx c)
              axes [v3/x-axis v3/y-axis v3/z-axis]]
          (loop [ax 0 best ##Inf n v3/z-axis]
            (if (>= ax 3)
              {:link link :p c :n n :depth (+ radius best)}
              (let [[best n] (if (< (nth dlo ax) best) [(nth dlo ax) (v3/neg (nth axes ax))] [best n])
                    [best n] (if (< (nth dhi ax) best) [(nth dhi ax) (nth axes ax)] [best n])]
                (recur (inc ax) best n)))))))

    :convex
    (let [poly (:poly obstacle)
          pt (convex/->poly [c])
          cv (convex/gjk-closest-vec pt poly)
          d (v3/length cv)]
      (if (> d 1e-6)
        (let [n (v3/scale cv (/ 1.0 d)) depth (- radius d)]
          (when (> depth (- slop)) {:link link :p (v3/sub c (v3/scale n radius)) :n n :depth depth}))
        (when-let [[pd n0] (convex/epa-penetration pt poly)]
          (let [centroid (v3/scale (reduce v3/add v3/zero (:verts poly)) (/ 1.0 (max 1 (count (:verts poly)))))
                n (if (< (v3/dot n0 (v3/sub c centroid)) 0.0) (v3/neg n0) n0)]
            {:link link :p c :n n :depth (+ radius pd)}))))))
