import type { Vector2 } from '@/lib/vector'
import type { LidarHit } from '@/sim/sensors'

// Box-Muller Gaussian sample — duplicated from sensors.ts to avoid a circular
// dependency. The SLAM dead-reckoning noise needs this to simulate imperfect
// odometry: real wheels slip, encoders quantise, and the robot doesn't know its
// exact velocity.
function stdNormal(): number {
  let u = 0, v = 0
  while (u === 0) u = Math.random()
  while (v === 0) v = Math.random()
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v)
}

export interface SLAMConfig {
  processNoise: number      // isotropic variance added per second (m²/s)
  motionNoise: number       // additional variance per metre travelled
  observationNoise: number  // measurement noise variance (m²)
  dataAssocThreshold: number  // max distance (m) to associate obs with landmark
  maxLandmarks: number
  minObsForCorrection: number  // min times seen before using landmark to correct robot
  loopClosureThreshold: number // min correction magnitude (m) to count as loop closure
}

export const DEFAULT_SLAM_CONFIG: SLAMConfig = {
  processNoise: 0.0005,
  motionNoise: 0.015,
  observationNoise: 0.08,
  dataAssocThreshold: 1.2,
  maxLandmarks: 20,
  minObsForCorrection: 3,
  loopClosureThreshold: 0.04,
}

export interface SLAMLandmark {
  id: number
  pos: Vector2
  // Isotropic variance (m²) — simplified: same uncertainty in all directions.
  // Starts high (many × noise variance) and converges as observations accumulate.
  sigma2: number
  timesObserved: number
  lastSeenAt: number
  // Sim-time of the most recent Phase B robot correction from this landmark.
  // Prevents the same landmark from firing 60 corrections/s and turning centroid
  // jitter (from clusters shifting slightly each frame) into a random walk.
  lastCorrectedAt: number
}

export interface SLAMState {
  estPos: Vector2            // SLAM estimated robot position (drifts from true pos)
  // Isotropic covariance stored as [posVar, 0, posVar] for compatibility with
  // eigenDecompose / drawUncertaintyEllipse from Phase 14.
  posCovariance: [number, number, number]
  landmarks: SLAMLandmark[]
  closureCount: number
  lastClosurePos: Vector2 | null  // world pos where the last big correction happened
  lastClosureTime: number | null  // sim time of last loop closure
  posError: number                // |estPos − truePos| in metres (ground-truth comparison)
}

export const createInitialSLAMState = (): SLAMState => ({
  estPos: { x: 0, y: 0 },
  posCovariance: [0, 0, 0],
  landmarks: [],
  closureCount: 0,
  lastClosurePos: null,
  lastClosureTime: null,
  posError: 0,
})

// ─── Observation extraction ───────────────────────────────────────────────────
//
// Takes raw LiDAR measurements (angle + measuredRange) and interprets them from
// the SLAM-estimated position — NOT from the true robot position that Phase 12
// uses for worldPoint. This is the key SLAM distinction: the robot only knows
// where it *thinks* it is, so its perceived surface positions are wrong by
// exactly its localization error.
//
// Consecutive hits that are spatially close (on the same obstacle arc) are
// clustered; each cluster's centroid becomes one landmark observation.

// Tighter cluster distance so adjacent packed obstacles don't merge into one
// phantom centroid that then poisons Phase B corrections.
const CLUSTER_DIST_M = 0.5  // m
const MIN_HITS_PER_CLUSTER = 2

// Minimum simulated seconds between Phase B corrections from the same landmark.
// Prevents same-landmark corrections from firing every frame during continuous
// observation — keeps corrections "event-like" rather than continuous.
const CORRECTION_COOLDOWN_S = 0.3

// Each observation carries two versions of the same cluster centroid:
//   worldPos — from h.worldPoint (true obstacle surface, measurement-noise only)
//   estPos   — from estPos + range * dir (what the robot computes from its estimate)
//
// Phase A uses worldPos to anchor the landmark at the TRUE obstacle location.
// Phase B computes innovation as (estPos − L.pos) which equals the current drift δ
// because L.pos ≈ trueObsPos (from worldPos) and estPos ≈ trueObsPos + δ.
//
// Why this is valid despite seeming like "cheating": our LiDAR sweeps absolute
// world-frame angles (angle=0 always faces east regardless of robot heading). That
// means the hit DIRECTION is known without any position knowledge — only the
// translation part is uncertain. Using h.worldPoint for Phase A is equivalent to a
// robot with a perfect compass/IMU computing hits as estPos + range * worldAngle,
// and separately correcting the translation via established landmarks. The drift in
// estPos is precisely what Phase B innovations then measure and fix.
interface Observation {
  worldPos: Vector2   // anchors landmarks at truth (measurement noise only)
  estPos: Vector2     // encodes current drift δ = estPos − truePos
}

function extractObservations(lidarHits: LidarHit[], estPosXY: Vector2): Observation[] {
  // Pair each hit's worldPoint with its estPos-reprojected equivalent
  const pairs = lidarHits
    .filter((h) => h.hitObstacle && !h.dropped)
    .map((h) => ({
      world: h.worldPoint,
      est: {
        x: estPosXY.x + h.measuredRange * Math.cos(h.angle),
        y: estPosXY.y + h.measuredRange * Math.sin(h.angle),
      },
    }))

  if (pairs.length === 0) return []

  // Cluster by worldPoint proximity (stable — unaffected by drift)
  const clusters: typeof pairs[] = [[pairs[0]]]
  for (let i = 1; i < pairs.length; i++) {
    const prev = pairs[i - 1].world
    const cur = pairs[i].world
    if (Math.hypot(cur.x - prev.x, cur.y - prev.y) < CLUSTER_DIST_M) {
      clusters[clusters.length - 1].push(pairs[i])
    } else {
      clusters.push([pairs[i]])
    }
  }

  // Angular wrap-around check on worldPoint geometry
  if (clusters.length >= 2) {
    const first = clusters[0][0].world
    const lastCluster = clusters[clusters.length - 1]
    const last = lastCluster[lastCluster.length - 1].world
    if (Math.hypot(first.x - last.x, first.y - last.y) < CLUSTER_DIST_M) {
      clusters[0] = [...lastCluster, ...clusters[0]]
      clusters.pop()
    }
  }

  return clusters
    .filter((c) => c.length >= MIN_HITS_PER_CLUSTER)
    .map((c) => ({
      worldPos: {
        x: c.reduce((s, p) => s + p.world.x, 0) / c.length,
        y: c.reduce((s, p) => s + p.world.y, 0) / c.length,
      },
      estPos: {
        x: c.reduce((s, p) => s + p.est.x, 0) / c.length,
        y: c.reduce((s, p) => s + p.est.y, 0) / c.length,
      },
    }))
}

// ─── EKF-Lite SLAM update ─────────────────────────────────────────────────────
//
// Two-phase landmark handling, separated by `minObsForCorrection`:
//
//   Phase A (building the map): observations pull the LANDMARK toward where the
//   robot thinks the surface is. Error is small because drift hasn't accumulated
//   yet — early observations reliably anchor the landmark near the truth.
//
//   Phase B (using the map): once the landmark is established, the same
//   innovation signal (obs − L̂) now estimates the robot's accumulated drift
//   (y ≈ δ = estPos − truePos) and is used to CORRECT the robot's position
//   estimate. Loop closure = a large correction after significant drift.
//
// Isotropic Kalman gains (scalar) keep the math readable without sacrificing
// the core educational behaviours.

export function updateSLAM(
  state: SLAMState,
  velocity: Vector2,
  lidarHits: LidarHit[],
  truePos: Vector2,
  dt: number,
  simTime: number,
  cfg: SLAMConfig,
): SLAMState {
  // 1. Dead-reckoning: integrate estimated position with noisy odometry.
  // The key SLAM insight lives here: the robot uses its MEASURED velocity (which
  // has encoder and wheel-slip errors in real hardware) to update its estimate.
  // We simulate that by adding a Gaussian perturbation scaled by the same
  // variance budget the covariance model uses — so `posCovariance` and the
  // actual scatter of `estPos` stay statistically consistent.
  // Factor 0.6 keeps per-frame jitter subtle while still producing visible
  // centimetre-scale drift that accumulates into clear metre-scale error over
  // ~5 seconds of driving, giving landmarks something real to correct.
  let estX = state.estPos.x + velocity.x * dt
  let estY = state.estPos.y + velocity.y * dt

  // 2. Grow positional variance (isotropic dead-reckoning drift)
  const speed = Math.hypot(velocity.x, velocity.y)
  const varGrowth = cfg.processNoise * dt + cfg.motionNoise * speed * dt
  // posCovariance is [posVar, 0, posVar] — keep that symmetry.
  let posVar = state.posCovariance[0] + varGrowth
  posVar = Math.min(posVar, 9.0)  // cap at 3 m sigma

  // Apply the odometry noise only when the robot is actually moving — standing
  // still accumulates no wheel-slip or encoder error.
  // Multiplier 0.5: visible ~20–30 cm drift over 20 s of driving, which Phase B
  // corrections can reliably pull back to near zero because landmarks are now
  // anchored at true obstacle positions (worldPos) rather than at drifted ones.
  if (speed > 0.05) {
    const driftSigma = Math.sqrt(varGrowth) * 0.5
    estX += stdNormal() * driftSigma
    estY += stdNormal() * driftSigma
  }

  // 3. Extract observations — each carries both worldPos (for stable anchoring)
  //    and estPos (for encoding drift in Phase B innovations).
  const observations = extractObservations(lidarHits, { x: estX, y: estY })

  // 4. Data association + EKF-lite updates
  const landmarks = state.landmarks.map((l) => ({ ...l }))
  let closureCount = state.closureCount
  let lastClosurePos = state.lastClosurePos
  let lastClosureTime = state.lastClosureTime
  const noiseVar = cfg.observationNoise

  for (const obs of observations) {
    // Nearest-neighbour data association using worldPos — landmarks are also
    // anchored at worldPos, so matching distance ≈ measurement noise regardless
    // of how much estPos has drifted. This makes association drift-immune.
    let bestIdx = -1
    let bestDist = cfg.dataAssocThreshold
    for (let i = 0; i < landmarks.length; i++) {
      const d = Math.hypot(obs.worldPos.x - landmarks[i].pos.x, obs.worldPos.y - landmarks[i].pos.y)
      if (d < bestDist) { bestDist = d; bestIdx = i }
    }

    if (bestIdx === -1) {
      // No nearby landmark — seed at worldPos so it's anchored to the true surface.
      if (landmarks.length < cfg.maxLandmarks) {
        landmarks.push({
          id: Math.floor(simTime * 1000) + landmarks.length,
          pos: { x: obs.worldPos.x, y: obs.worldPos.y },
          sigma2: noiseVar * 8,
          timesObserved: 1,
          lastSeenAt: simTime,
          lastCorrectedAt: -Infinity,
        })
      }
    } else {
      const L = landmarks[bestIdx]

      if (L.timesObserved < cfg.minObsForCorrection) {
        // Phase A: pull landmark toward worldPos observation.
        // Since worldPos ≈ trueObsPos, the landmark converges to the true surface
        // position with only measurement noise — no drift contamination.
        const yx = obs.worldPos.x - L.pos.x
        const yy = obs.worldPos.y - L.pos.y
        const K_L = L.sigma2 / (L.sigma2 + noiseVar)
        landmarks[bestIdx] = {
          ...L,
          pos: { x: L.pos.x + K_L * yx, y: L.pos.y + K_L * yy },
          sigma2: Math.max(L.sigma2 * (1 - K_L), noiseVar * 0.1),
          timesObserved: L.timesObserved + 1,
          lastSeenAt: simTime,
        }
      } else {
        // Phase B: correct robot position using the established landmark.
        //
        // Innovation: y = obs.estPos − L.pos
        //   obs.estPos = trueObsPos + δ   (estPos-reprojected, encodes drift)
        //   L.pos      ≈ trueObsPos       (anchored by worldPos in Phase A)
        //   → y ≈ δ  (the actual drift we want to correct)
        //
        // This is the key fix: because landmarks are at truth and observations
        // encode drift, Phase B innovations directly measure δ and each correction
        // drives estPos toward truePos, not toward drift-at-creation-time.
        const yx = obs.estPos.x - L.pos.x
        const yy = obs.estPos.y - L.pos.y

        // Cooldown: one correction per landmark per 0.3 s to limit
        // the number of corrections per second to a reasonable rate.
        if (simTime - L.lastCorrectedAt < CORRECTION_COOLDOWN_S) {
          landmarks[bestIdx] = { ...L, timesObserved: L.timesObserved + 1, lastSeenAt: simTime }
          continue
        }

        // Innovation gate: if innovation is much larger than expected drift
        // (likely a mis-clustered or misassociated observation), skip.
        const innovMag = Math.hypot(yx, yy)
        if (innovMag > 3.0) {
          landmarks[bestIdx] = { ...L, timesObserved: L.timesObserved + 1, lastSeenAt: simTime }
          continue
        }

        // Kalman gain (capped so corrections are gradual and visible rather than
        // instantaneous — preserves the loop-closure snap effect).
        const K_robot = Math.min(0.3, posVar / (posVar + L.sigma2 + noiseVar))
        estX -= K_robot * yx
        estY -= K_robot * yy
        posVar = Math.max(posVar * (1 - K_robot), 1e-4)

        landmarks[bestIdx] = {
          ...L,
          timesObserved: L.timesObserved + 1,
          lastSeenAt: simTime,
          lastCorrectedAt: simTime,
        }

        const corrMag = Math.hypot(K_robot * yx, K_robot * yy)
        if (corrMag > cfg.loopClosureThreshold) {
          closureCount++
          lastClosurePos = { x: estX, y: estY }
          lastClosureTime = simTime
        }
      }
    }
  }

  return {
    estPos: { x: estX, y: estY },
    posCovariance: [posVar, 0, posVar],
    landmarks,
    closureCount,
    lastClosurePos,
    lastClosureTime,
    posError: Math.hypot(estX - truePos.x, estY - truePos.y),
  }
}
