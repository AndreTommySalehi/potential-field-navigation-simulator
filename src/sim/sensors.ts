import type { Vector2 } from '@/lib/vector'
import type { Obstacle } from '@/sim/forces'

export interface SensorConfig {
  lidar: {
    enabled: boolean
    numRays: number     // 8–360
    range: number       // max sensing range in metres
    rangeNoise: number  // std dev of Gaussian noise added to each range reading (metres)
    dropoutRate: number // probability [0,1] that a reading is silently discarded
  }
}

export const DEFAULT_SENSOR_CONFIG: SensorConfig = {
  lidar: {
    enabled: false,
    numRays: 36,
    range: 5,
    rangeNoise: 0.15,
    dropoutRate: 0.05,
  },
}

export interface LidarHit {
  angle: number         // absolute world angle of this ray (radians)
  trueRange: number     // ground-truth distance to the obstacle surface (or maxRange if no hit)
  measuredRange: number // noisy measured distance (always <= maxRange)
  hitObstacle: boolean  // true if this ray struck an obstacle within range
  dropped: boolean      // true if this reading was lost (dropout)
  worldPoint: Vector2   // endpoint of the measured ray in world coords
}

/** Box-Muller: one standard-normal sample. */
function stdNormal(): number {
  const u1 = Math.max(1e-10, Math.random())
  const u2 = Math.random()
  return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2)
}

/**
 * Ray-circle intersection.
 * Returns the positive t (distance along the unit ray) of the *first* surface
 * hit, or null if the ray misses or the intersection is behind the origin.
 */
function rayCircleT(
  origin: Vector2,
  dir: Vector2, // must be a unit vector
  center: Vector2,
  radius: number,
): number | null {
  const dx = origin.x - center.x
  const dy = origin.y - center.y
  // Reduced quadratic: t² + 2bt + c = 0, where a=1 (unit direction)
  const b = dx * dir.x + dy * dir.y
  const c = dx * dx + dy * dy - radius * radius
  const disc = b * b - c
  if (disc < 0) return null
  const sqrtD = Math.sqrt(disc)
  const t1 = -b - sqrtD // entry point (smaller t)
  const t2 = -b + sqrtD // exit point
  if (t1 > 1e-4) return t1
  if (t2 > 1e-4) return t2
  return null
}

/**
 * Simulate one full LiDAR sweep from `robotPos`.
 * Rays are cast at uniform angular intervals over 360°, starting from angle 0
 * (east) — for a full scan the starting angle doesn't change what's measured,
 * only which ray index faces which direction.
 */
export function simulateLidar(
  robotPos: Vector2,
  _robotAngle: number, // reserved; full-scan so starting offset doesn't matter
  obstacles: Obstacle[],
  config: SensorConfig['lidar'],
): LidarHit[] {
  const hits: LidarHit[] = []
  const angleStep = (2 * Math.PI) / config.numRays

  for (let i = 0; i < config.numRays; i++) {
    const angle = i * angleStep
    const dir: Vector2 = { x: Math.cos(angle), y: Math.sin(angle) } // world coords: +y is up, standard math convention

    // Find the nearest obstacle surface along this ray
    let nearestT = config.range
    let hitObstacle = false

    for (const obs of obstacles) {
      const t = rayCircleT(robotPos, dir, obs.position, obs.radius)
      if (t !== null && t < nearestT) {
        nearestT = t
        hitObstacle = true
      }
    }

    const trueRange = nearestT

    // Apply dropout — silently lose this reading
    if (Math.random() < config.dropoutRate) {
      hits.push({
        angle,
        trueRange,
        measuredRange: config.range,
        hitObstacle,
        dropped: true,
        worldPoint: {
          x: robotPos.x + config.range * dir.x,
          y: robotPos.y + config.range * dir.y,
        },
      })
      continue
    }

    // Add Gaussian range noise only to genuine hits (noise on a free-space ray
    // would move the apparent endpoint into thin air, which isn't physically
    // meaningful and confuses the APF).
    const noise = hitObstacle ? stdNormal() * config.rangeNoise : 0
    const measured = Math.max(0.05, Math.min(trueRange + noise, config.range))

    hits.push({
      angle,
      trueRange,
      measuredRange: measured,
      hitObstacle,
      dropped: false,
      worldPoint: {
        x: robotPos.x + measured * dir.x,
        y: robotPos.y + measured * dir.y,
      },
    })
  }

  return hits
}

/**
 * Convert LiDAR hit points into virtual point obstacles for APF computation.
 * Each surface hit becomes a small obstacle centred at the measured hit point.
 * Using measured (not true) positions ensures APF forces reflect what the robot
 * actually perceives — the noise carries through into the navigation decisions.
 */
export function hitsToPerceivedObstacles(hits: LidarHit[]): Obstacle[] {
  return hits
    .filter((h) => h.hitObstacle && !h.dropped)
    .map((h) => ({ position: h.worldPoint, radius: 0.15 }))
}
