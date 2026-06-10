import type { Vector2 } from '@/lib/vector'

export interface UncertaintyConfig {
  processNoise: number    // isotropic variance added per second (m²/s) — background drift
  motionNoise: number     // additional variance per metre travelled — along-motion drift
  sensorCorrection: boolean
  correctionRate: number  // exponential decay rate (1/s) when sensor sees obstacles
  minSigma: number        // minimum 1-σ radius (m) — floor from sensor accuracy
  maxSigma: number        // maximum 1-σ cap so the ellipse stays on screen
}

export const DEFAULT_UNCERTAINTY_CONFIG: UncertaintyConfig = {
  processNoise: 0.005,
  motionNoise: 0.05,
  sensorCorrection: true,
  correctionRate: 1.5,
  minSigma: 0.05,
  maxSigma: 3.0,
}

// Symmetric 2×2 covariance stored as [P00, P01, P11].
export type Covariance2 = [number, number, number]
export const INITIAL_COVARIANCE: Covariance2 = [0, 0, 0]

export interface UncertaintyStats {
  P: Covariance2
  sigma1: number   // 1-σ major semi-axis (m)
  sigma2: number   // 1-σ minor semi-axis (m)
  angle: number    // major-axis angle in world space (rad, +y-up convention)
}

/**
 * Eigendecompose a 2×2 symmetric matrix.
 * Returns eigenvalues (λ1 ≥ λ2) and the angle of the major eigenvector
 * in world space (+y up).
 */
export function eigenDecompose(P00: number, P01: number, P11: number): {
  λ1: number; λ2: number; angle: number
} {
  const traceHalf = (P00 + P11) * 0.5
  const disc = Math.sqrt(((P00 - P11) * 0.5) ** 2 + P01 * P01)
  const λ1 = traceHalf + disc
  const λ2 = Math.max(0, traceHalf - disc)

  // Major eigenvector: unnormalised [P01, λ1 - P00], i.e. atan2(λ1-P00, P01).
  // Special-case when P01 ≈ 0 to avoid atan2(0, 0).
  const angle = Math.abs(P01) < 1e-9
    ? (P00 >= P11 ? 0 : Math.PI * 0.5)
    : Math.atan2(λ1 - P00, P01)

  return { λ1: Math.max(0, λ1), λ2, angle }
}

/**
 * EKF prediction step: grow covariance by one timestep of dead-reckoning noise.
 *
 * Two additive noise terms mirror real odometry error sources:
 *   - isotropic base noise (sensor clock jitter, round-off in encoders)
 *   - along-motion noise (wheel slip, encoder quantisation in the travel direction)
 */
export function propagateCovariance(
  P: Covariance2,
  vel: Vector2,
  dt: number,
  cfg: UncertaintyConfig,
): Covariance2 {
  const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y)
  const qBase = cfg.processNoise * dt
  const qVel = cfg.motionNoise * speed * dt

  let dP00 = qBase
  let dP01 = 0
  let dP11 = qBase
  if (speed > 1e-6) {
    const vx = vel.x / speed
    const vy = vel.y / speed
    dP00 += qVel * vx * vx
    dP01 += qVel * vx * vy
    dP11 += qVel * vy * vy
  }

  const maxVar = cfg.maxSigma * cfg.maxSigma
  return [
    Math.min(P[0] + dP00, maxVar),
    P[1] + dP01,
    Math.min(P[2] + dP11, maxVar),
  ]
}

/**
 * EKF correction step (simplified): exponential decay toward the sensor accuracy floor.
 * Triggered when the LiDAR actively sees obstacles — observing features corrects drift.
 */
export function correctCovariance(
  P: Covariance2,
  dt: number,
  cfg: UncertaintyConfig,
): Covariance2 {
  const decay = Math.exp(-cfg.correctionRate * dt)
  const minVar = cfg.minSigma * cfg.minSigma
  return [
    Math.max(P[0] * decay, minVar),
    P[1] * decay,
    Math.max(P[2] * decay, minVar),
  ]
}

/** Derive display stats from a raw covariance matrix. */
export function covToStats(P: Covariance2): UncertaintyStats {
  const { λ1, λ2, angle } = eigenDecompose(P[0], P[1], P[2])
  return { P, sigma1: Math.sqrt(λ1), sigma2: Math.sqrt(λ2), angle }
}
