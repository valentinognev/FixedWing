import { describe, expect, it } from 'vitest'
import { layoutDiagram } from '../diagramLayout'
import type { DiagramLoop } from '../types'

function node(id: string, kind: DiagramLoop['nodes'][number]['kind'], label: string, text = '') {
  return { id, kind, label, text }
}

const PITCH: DiagramLoop[] = [
  {
    input: 'theta',
    output: 'pitch',
    feedback: 'theta',
    nodes: [
      node('theta_ref', 'input', 'theta_ref'),
      node('sum_e', 'sum', 'sum'),
      node('pi', 'block', 'PI', '2 + 0.5/s'),
      node('kq', 'block', 'kq', '0.5'),
      node('sum_rate', 'sum', 'sum'),
      node('disturbance', 'block', 'disturbance', '+1 deg'),
      node('sum_out', 'sum', 'sum'),
      node('plant', 'plant', 'airframe'),
    ],
    wires: [
      { from: 'theta_ref', to: 'sum_e', sign: '+' },
      { from: 'plant', to: 'sum_e', sign: '-' },
      { from: 'sum_e', to: 'pi', sign: '' },
      { from: 'pi', to: 'sum_rate', sign: '+' },
      { from: 'plant', to: 'kq', sign: '' },
      { from: 'kq', to: 'sum_rate', sign: '-' },
      { from: 'disturbance', to: 'sum_out', sign: '+' },
      { from: 'sum_rate', to: 'sum_out', sign: '+' },
      { from: 'sum_out', to: 'plant', sign: '' },
    ],
  },
]

describe('diagram return lanes', () => {
  it('routes both pitch returns under the row and keeps the forward wire on the edges', () => {
    const layout = layoutDiagram(PITCH)
    const sumE = layout.boxes.find((box) => box.key === '0:sum_e')
    const theta = layout.boxes.find((box) => box.key === '0:theta_ref')
    const plant = layout.boxes.find((box) => box.key === 'plant')
    expect(sumE).toBeDefined()
    expect(theta).toBeDefined()
    expect(plant).toBeDefined()
    if (sumE === undefined || theta === undefined || plant === undefined) return

    const returns = layout.wires.filter((wire) => wire.feedback)
    expect(returns.map((wire) => `${wire.from}->${wire.to}`)).toEqual([
      'plant->sum_e',
      'plant->kq',
    ])
    const laneY = returns.map((wire) => wire.points.find((point) => point.y > sumE.y + sumE.h)?.y)
    expect(laneY[1]! - laneY[0]!).toBe(16)
    expect(laneY[0]).toBeGreaterThan(sumE.y + sumE.h)

    const elbow = returns[0].points.find((point) => point.x === plant.x - 14)
    expect(elbow).toBeDefined()
    const rightmost = Math.max(
      ...layout.boxes.filter((box) => box.key !== 'plant').map((box) => box.x + box.w),
    )
    expect(plant.x - 14).toBeGreaterThan(rightmost)

    const forward = layout.wires.find((wire) => wire.from === 'theta_ref' && wire.to === 'sum_e')
    expect(forward?.points).toEqual([
      { x: theta.x + theta.w, y: theta.y + theta.h / 2 },
      { x: sumE.x, y: sumE.y + sumE.h / 2 },
    ])
    expect(returns[0].signAt?.y).toBeGreaterThan(sumE.y + sumE.h)
    const rateSign = layout.wires.find((wire) => wire.from === 'kq' && wire.to === 'sum_rate')
    const sumRate = layout.boxes.find((box) => box.key === '0:sum_rate')
    expect(rateSign?.signAt?.x).toBeLessThan(sumRate!.x)
  })

  it('drops a lower loop’s airframe return in the gap beside the airframe', () => {
    const second: DiagramLoop = {
      input: 'vt',
      output: 'throttle',
      feedback: 'vt',
      nodes: [
        node('sum_rate', 'sum', 'sum'),
        node('plant', 'plant', 'airframe'),
      ],
      wires: [{ from: 'plant', to: 'sum_rate', sign: '-' }],
    }
    const layout = layoutDiagram([PITCH[0], second])
    const plant = layout.boxes.find((box) => box.key === 'plant')!
    const lower = layout.boxes.find((box) => box.key === '1:sum_rate')!
    const upper = layout.boxes.filter((box) => box.key.startsWith('0:') || box.key === 'plant')
    const wire = layout.wires.find((item) => item.from === 'plant' && item.to === 'sum_rate')!
    expect(wire.feedback).toBe(true)
    expect(wire.points.some((point) => point.x === plant.x - 14)).toBe(true)
    const rightmost = Math.max(
      ...layout.boxes.filter((box) => box.key !== 'plant').map((box) => box.x + box.w),
    )
    expect(plant.x - 14).toBeGreaterThan(rightmost)
    const lane = wire.points.find((point) => point.y > lower.y + lower.h)!
    expect(lane.y).toBeGreaterThan(lower.y + lower.h)
    for (const box of upper) {
      if (box.key === 'plant') continue
      expect(lane.y).toBeGreaterThan(box.y + box.h)
    }
  })
})
