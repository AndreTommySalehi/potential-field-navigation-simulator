import { type Vector2, add, distance, limit, magnitude, normalize, scale, sub, sum, zero } from '@/lib/vector'
import type { RobotState } from './simulationEngine'

export interface Obstacle {
  position: Vector2
  radius: number
}

// obstacleInfluenceRadius is Q* in Khatib's notation below — how far past an
// obstacle's surface its repulsive field still reaches.
export interface SimulationParams {
  attractiveGain: number
  repulsiveGain: number
  obstacleInfluenceRadius: number
  damping: number
  maxSpeed: number
}

export const DEFAULT_PARAMS: SimulationParams = {
  attractiveGain: 1.5,
  repulsiveGain: 3.5,
  obstacleInfluenceRadius: 1.8,
  damping: 1.2,
  maxSpeed: 5,
}

// Repulsion grows as 1/d^2 near a surface, so a close pass can otherwise blow
// past anything the integrator can handle at normal timesteps.
const MAX_REPULSIVE_FORCE = 60

export interface ForceBreakdown {
  attractive: Vector2
  repulsive: Vector2
  damping: Vector2
  net: Vector2
}

// U = 1/2 k d^2, so F = -∇U = k(goal - position) — a spring pulling toward the goal.
export const attractiveForce = (position: Vector2, goal: Vector2, gain: number): Vector2 =>
  scale(sub(goal, position), gain)

// Intermediate potential: the gradient is η(1/d − 1/Q*)/Q* · d̂.
// Keeping the Khatib (1/d − 1/Q*) numerator gives a 1/d singularity near
// the surface so the robot can't push through — capped by MAX_REPULSIVE_FORCE
// below, but that cap only bites in the last few millimetres, not across the
// whole zone. Dividing by Q* instead of d² makes the mid-range and outer force
// ~10–30× larger than original Khatib, so the influence ring has a real,
// visible effect on the robot's path: the robot starts curving from the moment
// it enters the dashed ring, not only when it's nearly touching the surface.
//
// Matching scalar potential: U = η/Q* · (ln(Q*/d) − 1 + d/Q*)
// (zero at d = Q*, diverges logarithmically toward the surface — same idea as
// the force, smooth and bounded in log-space rather than polynomial-space).
export const repulsiveForce = (
  position: Vector2,
  obstacle: Obstacle,
  gain: number,
  influenceRadius: number,
): Vector2 => {
  const offset = sub(position, obstacle.position)
  const surfaceDistance = Math.max(magnitude(offset) - obstacle.radius, 1e-3)
  if (surfaceDistance >= influenceRadius) return zero()

  const strength = (gain * (1 / surfaceDistance - 1 / influenceRadius)) / influenceRadius
  return scale(normalize(offset), strength)
}

export const totalRepulsiveForce = (
  position: Vector2,
  obstacles: Obstacle[],
  gain: number,
  influenceRadius: number,
): Vector2 =>
  limit(
    sum(obstacles.map((obstacle) => repulsiveForce(position, obstacle, gain, influenceRadius))),
    MAX_REPULSIVE_FORCE,
  )

// Scalar potentials — the same U each force above is the negative gradient
// of. Heatmaps (Phase 6) shade these directly rather than the forces, since
// "low potential = good place to be" reads more intuitively as color than
// vector magnitude does.
const attractivePotential = (position: Vector2, goal: Vector2, gain: number): number =>
  0.5 * gain * distance(position, goal) ** 2

// Exported so the heatmap renderer can normalize the attractive and repulsive
// contributions independently — the bounded repulsive formula (max η/2) is
// otherwise invisible against the quadratic attractive bowl's large range.
export const attractivePotentialAt = (
  position: Vector2,
  goal: Vector2 | null,
  params: SimulationParams,
): number => (goal ? attractivePotential(position, goal, params.attractiveGain) : 0)

// Scalar potential matching the intermediate force above: U = η/Q*(ln(Q*/d) − 1 + d/Q*)
// Zero at d = Q*, grows logarithmically toward the surface — guaranteed consistent
// with the force since this is exactly the antiderivative of −F_magnitude.
const repulsivePotential = (position: Vector2, obstacle: Obstacle, gain: number, influenceRadius: number): number => {
  const surfaceDistance = Math.max(distance(position, obstacle.position) - obstacle.radius, 1e-3)
  if (surfaceDistance >= influenceRadius) return 0
  return (gain / influenceRadius) * (Math.log(influenceRadius / surfaceDistance) - 1 + surfaceDistance / influenceRadius)
}

export const potentialAt = (
  position: Vector2,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
): number => {
  const attractive = goal ? attractivePotential(position, goal, params.attractiveGain) : 0
  const repulsive = obstacles.reduce(
    (total, obstacle) =>
      total + repulsivePotential(position, obstacle, params.repulsiveGain, params.obstacleInfluenceRadius),
    0,
  )
  return attractive + repulsive
}

// Pure field math, no damping — damping depends on the robot's velocity, not
// its position, so it has no place in a static field. This is what Phase 5's
// grid overlay and Phase 6's heatmap both sample.
export const fieldForceAt = (
  position: Vector2,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
): Vector2 => {
  const attractive = goal ? attractiveForce(position, goal, params.attractiveGain) : zero()
  const repulsive = totalRepulsiveForce(position, obstacles, params.repulsiveGain, params.obstacleInfluenceRadius)
  return add(attractive, repulsive)
}

// Returns the breakdown rather than just the total so the UI can draw each
// contributor as its own arrow.
export const computeForces = (
  robot: RobotState,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
): ForceBreakdown => {
  const attractive = goal ? attractiveForce(robot.position, goal, params.attractiveGain) : zero()
  const repulsive = totalRepulsiveForce(robot.position, obstacles, params.repulsiveGain, params.obstacleInfluenceRadius)
  const damping = scale(robot.velocity, -params.damping)
  return { attractive, repulsive, damping, net: add(add(attractive, repulsive), damping) }
}
