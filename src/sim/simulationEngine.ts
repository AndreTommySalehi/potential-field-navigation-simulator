import { type Vector2, add, limit, scale, sum, zero } from '@/lib/vector'
import { computeForces, DEFAULT_PARAMS, type Obstacle, type SimulationParams } from './forces'

export interface RobotState {
  position: Vector2
  velocity: Vector2
}

export interface SimulationState {
  robot: RobotState
  goal: Vector2 | null
  obstacles: Obstacle[]
  time: number
  stepCount: number
}

export const createInitialRobot = (): RobotState => ({
  position: { x: -5, y: -2.5 },
  velocity: zero(),
})

export const createInitialState = (): SimulationState => ({
  robot: createInitialRobot(),
  goal: null,
  obstacles: [],
  time: 0,
  stepCount: 0,
})

export type IntegrationMethod = 'euler' | 'semi-implicit-euler' | 'rk4'

export const INTEGRATION_METHODS: { id: IntegrationMethod; label: string; blurb: string }[] = [
  {
    id: 'euler',
    label: 'Euler',
    blurb: 'Moves with the old velocity, then updates it — cheap, but tends to overshoot and pick up energy it shouldn’t have.',
  },
  {
    id: 'semi-implicit-euler',
    label: 'Semi-implicit Euler',
    blurb: 'Updates velocity first and moves with that — same cost as Euler, far more stable for spring-like forces. What this sim has used since Phase 1.',
  },
  {
    id: 'rk4',
    label: 'RK4',
    blurb: 'Samples the slope four times across the step and blends them — the most accurate of the three, at roughly 4x the work per step.',
  },
]

// d/dt of robot state: position changes by velocity, velocity changes by
// acceleration (= net force, since mass is 1). Every integrator below is just
// a different recipe for combining samples of this.
interface Derivative {
  velocity: Vector2
  acceleration: Vector2
}

const derivativeAt = (
  robot: RobotState,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
  extraForce?: Vector2,
): Derivative => {
  const { net } = computeForces(robot, goal, obstacles, params)
  return { velocity: robot.velocity, acceleration: extraForce ? add(net, extraForce) : net }
}

// Textbook explicit Euler — moves using the velocity it had at the *start*
// of the step. That one-step lag is exactly what lets it gain energy it
// shouldn't on stiff springs (try cranking attractiveGain and watch it
// orbit the goal instead of settling into it).
const eulerStep = (
  robot: RobotState,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
  dt: number,
  extraForce?: Vector2,
): RobotState => {
  const { velocity, acceleration } = derivativeAt(robot, goal, obstacles, params, extraForce)
  return {
    position: add(robot.position, scale(velocity, dt)),
    velocity: limit(add(robot.velocity, scale(acceleration, dt)), params.maxSpeed),
  }
}

// Same cost as Euler, but flips the order: update velocity, then move using
// *that* — borrowing a bit of information from the end of the step is enough
// to make it symplectic (energy-conserving) for spring-like forces, which is
// why it's been the sim's default since Phase 1.
const semiImplicitEulerStep = (
  robot: RobotState,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
  dt: number,
  extraForce?: Vector2,
): RobotState => {
  const { net } = computeForces(robot, goal, obstacles, params)
  const total = extraForce ? add(net, extraForce) : net
  const velocity = limit(add(robot.velocity, scale(total, dt)), params.maxSpeed)
  const position = add(robot.position, scale(velocity, dt))
  return { position, velocity }
}

// Classic RK4: sample the derivative at the start, twice at the midpoint
// (using the previous sample to step there), and once at the end, then take
// a weighted average — (k1 + 2k2 + 2k3 + k4) / 6. Four force evaluations
// per step buys a much closer approximation of the true curve than either
// Euler variant gets from one.
const rk4Step = (
  robot: RobotState,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
  dt: number,
  extraForce?: Vector2,
): RobotState => {
  const advanced = (derivative: Derivative, fraction: number): RobotState => ({
    position: add(robot.position, scale(derivative.velocity, dt * fraction)),
    velocity: add(robot.velocity, scale(derivative.acceleration, dt * fraction)),
  })

  const k1 = derivativeAt(robot, goal, obstacles, params, extraForce)
  const k2 = derivativeAt(advanced(k1, 0.5), goal, obstacles, params, extraForce)
  const k3 = derivativeAt(advanced(k2, 0.5), goal, obstacles, params, extraForce)
  const k4 = derivativeAt(advanced(k3, 1), goal, obstacles, params, extraForce)

  const blend = (pick: (d: Derivative) => Vector2) =>
    scale(sum([pick(k1), scale(pick(k2), 2), scale(pick(k3), 2), pick(k4)]), 1 / 6)

  const velocity = limit(add(robot.velocity, scale(blend((d) => d.acceleration), dt)), params.maxSpeed)
  const position = add(robot.position, scale(blend((d) => d.velocity), dt))
  return { position, velocity }
}

const INTEGRATORS: Record<
  IntegrationMethod,
  (robot: RobotState, goal: Vector2 | null, obstacles: Obstacle[], params: SimulationParams, dt: number, extraForce?: Vector2) => RobotState
> = {
  euler: eulerStep,
  'semi-implicit-euler': semiImplicitEulerStep,
  rk4: rk4Step,
}

export const integrateRobot = (
  robot: RobotState,
  goal: Vector2 | null,
  obstacles: Obstacle[],
  params: SimulationParams,
  dt: number,
  method: IntegrationMethod,
  extraForce?: Vector2,
): RobotState => INTEGRATORS[method](robot, goal, obstacles, params, dt, extraForce)

export const stepSimulation = (
  state: SimulationState,
  dt: number,
  params: SimulationParams = DEFAULT_PARAMS,
  method: IntegrationMethod = 'semi-implicit-euler',
): SimulationState => ({
  robot: integrateRobot(state.robot, state.goal, state.obstacles, params, dt, method),
  goal: state.goal,
  obstacles: state.obstacles,
  time: state.time + dt,
  stepCount: state.stepCount + 1,
})
