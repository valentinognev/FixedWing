import { createElement } from 'react'
import { describe, expect, it } from 'vitest'
import { render } from '@testing-library/react'
import { ControllerDiagram } from '../components/ControllerDiagram'
import { layoutDiagram, type DiagramBox } from '../diagramLayout'
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
      { from: 'theta_ref', to: 'sum_e', sign: '+', signal: 'theta' },
      { from: 'plant', to: 'sum_e', sign: '-', signal: 'theta' },
      { from: 'sum_e', to: 'pi', sign: '', signal: 'e_theta' },
      { from: 'pi', to: 'sum_rate', sign: '+', signal: 'pitch' },
      { from: 'plant', to: 'kq', sign: '', signal: 'q' },
      { from: 'kq', to: 'sum_rate', sign: '-' },
      { from: 'disturbance', to: 'sum_out', sign: '+' },
      { from: 'sum_rate', to: 'sum_out', sign: '+', signal: 'pitch' },
      { from: 'sum_out', to: 'plant', sign: '', signal: 'pitch' },
    ],
  },
]

function hitsBox(
  points: { x: number; y: number }[],
  box: { x: number; y: number; w: number; h: number },
) {
  for (let index = 0; index < points.length - 1; index += 1) {
    const a = points[index]
    const b = points[index + 1]
    if (a.y === b.y) {
      if (a.y <= box.y || a.y >= box.y + box.h) continue
      const x1 = Math.min(a.x, b.x)
      const x2 = Math.max(a.x, b.x)
      if (x2 > box.x && x1 < box.x + box.w) return true
    } else if (a.x === b.x) {
      if (a.x <= box.x || a.x >= box.x + box.w) continue
      const y1 = Math.min(a.y, b.y)
      const y2 = Math.max(a.y, b.y)
      if (y2 > box.y && y1 < box.y + box.h) return true
    }
  }
  return false
}

function segmentDistance(
  point: { x: number; y: number },
  a: { x: number; y: number },
  b: { x: number; y: number },
) {
  const dx = b.x - a.x
  const dy = b.y - a.y
  const len2 = dx * dx + dy * dy
  if (len2 === 0) return Math.hypot(point.x - a.x, point.y - a.y)
  const t = Math.max(0, Math.min(1, ((point.x - a.x) * dx + (point.y - a.y) * dy) / len2))
  return Math.hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy))
}

function orient(
  a: { x: number; y: number },
  b: { x: number; y: number },
  c: { x: number; y: number },
) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

/** Length of a collinear run two segments share. A shared corner is zero. */
function collinearOverlap(
  a: { x: number; y: number },
  b: { x: number; y: number },
  c: { x: number; y: number },
  d: { x: number; y: number },
) {
  if (Math.hypot(b.x - a.x, b.y - a.y) < 0.5 || Math.hypot(d.x - c.x, d.y - c.y) < 0.5) return 0
  const horizontal = a.y === b.y && c.y === d.y && Math.abs(a.y - c.y) < 0.5
  const vertical = a.x === b.x && c.x === d.x && Math.abs(a.x - c.x) < 0.5
  if (!horizontal && !vertical) return 0
  const [a0, a1, b0, b1] = horizontal ? [a.x, b.x, c.x, d.x] : [a.y, b.y, c.y, d.y]
  const lo = Math.max(Math.min(a0, a1), Math.min(b0, b1))
  const hi = Math.min(Math.max(a0, a1), Math.max(b0, b1))
  return hi - lo
}

function boxesOverlap(a: DiagramBox, b: DiagramBox) {
  return a.x < b.x + b.w - 0.5 && a.x + a.w > b.x + 0.5 && a.y < b.y + b.h - 0.5 && a.y + a.h > b.y + 0.5
}

/** No stacked blocks, no wire through a block, no two wires drawn as one. */
function assertSeparate(layout: ReturnType<typeof layoutDiagram>) {
  for (let i = 0; i < layout.boxes.length; i += 1) {
    for (let j = i + 1; j < layout.boxes.length; j += 1) {
      expect(boxesOverlap(layout.boxes[i], layout.boxes[j])).toBe(false)
    }
  }
  for (const wire of layout.wires) {
    for (const box of layout.boxes) expect(hitsBox(wire.points, box)).toBe(false)
    for (const point of wire.points) {
      expect(point.x).toBeGreaterThanOrEqual(-0.5)
      expect(point.y).toBeGreaterThanOrEqual(-0.5)
      expect(point.x).toBeLessThanOrEqual(layout.width + 0.5)
      expect(point.y).toBeLessThanOrEqual(layout.height + 0.5)
    }
  }
  for (let i = 0; i < layout.wires.length; i += 1) {
    for (let j = i + 1; j < layout.wires.length; j += 1) {
      const left = layout.wires[i].points
      const right = layout.wires[j].points
      for (let a = 0; a < left.length - 1; a += 1) {
        for (let b = 0; b < right.length - 1; b += 1) {
          expect(segmentsCross(left[a], left[a + 1], right[b], right[b + 1])).toBe(false)
          expect(collinearOverlap(left[a], left[a + 1], right[b], right[b + 1])).toBeLessThanOrEqual(1)
        }
      }
    }
  }
}
function segmentsCross(
  a: { x: number; y: number },
  b: { x: number; y: number },
  c: { x: number; y: number },
  d: { x: number; y: number },
) {
  const ab = orient(a, b, c) * orient(a, b, d)
  const cd = orient(c, d, a) * orient(c, d, b)
  return ab < 0 && cd < 0
}

function wireDistance(point: { x: number; y: number }, points: { x: number; y: number }[]) {
  let best = Infinity
  for (let index = 0; index < points.length - 1; index += 1) {
    best = Math.min(best, segmentDistance(point, points[index], points[index + 1]))
  }
  return best
}

describe('diagram return lanes', () => {
  it('keeps the forward path on one straight line and lifts the disturbance above it', () => {
    const layout = layoutDiagram(PITCH)
    const keys = ['0:theta_ref', '0:sum_e', '0:pi', '0:sum_rate', '0:sum_out', 'plant']
    const boxes = keys.map((key) => layout.boxes.find((box) => box.key === key))
    expect(boxes.every((box) => box !== undefined)).toBe(true)
    if (boxes.some((box) => box === undefined)) return
    const row = boxes as NonNullable<(typeof boxes)[number]>[]
    expect(new Set(row.map((box) => box.y)).size).toBe(1)

    const mid = row[0].y + row[0].h / 2
    const pairs = [
      ['theta_ref', 'sum_e'],
      ['sum_e', 'pi'],
      ['pi', 'sum_rate'],
      ['sum_rate', 'sum_out'],
      ['sum_out', 'plant'],
    ]
    for (const [from, to] of pairs) {
      const wire = layout.wires.find((item) => item.from === from && item.to === to)
      expect(wire?.points.every((point) => point.y === mid)).toBe(true)
    }

    const disturbance = layout.boxes.find((box) => box.key === '0:disturbance')!
    const sumOut = row[4]
    expect(disturbance.y + disturbance.h).toBeLessThanOrEqual(sumOut.y - 24)
    const drop = layout.wires.find((wire) => wire.from === 'disturbance' && wire.to === 'sum_out')!
    const end = drop.points[drop.points.length - 1]
    expect(end.y).toBeLessThanOrEqual(sumOut.y + 1)
    expect(end.x).toBeGreaterThan(sumOut.x)
    expect(end.x).toBeLessThan(sumOut.x + sumOut.w)

    const kq = layout.boxes.find((box) => box.key === '0:kq')!
    expect(kq.y).toBeGreaterThanOrEqual(row[0].y + row[0].h)
    const fromPi = layout.wires.find((wire) => wire.from === 'pi' && wire.to === 'sum_rate')!
    expect(hitsBox(fromPi.points, kq)).toBe(false)
    expect(hitsBox(fromPi.points, disturbance)).toBe(false)
  })

  it('separates the return lanes and keeps every sum sign off its wire', () => {
    const layout = layoutDiagram(PITCH)
    const sumE = layout.boxes.find((box) => box.key === '0:sum_e')!
    const plant = layout.boxes.find((box) => box.key === 'plant')!
    const returns = layout.wires.filter((wire) => wire.feedback)
    expect(returns.map((wire) => `${wire.from}->${wire.to}`)).toEqual([
      'plant->sum_e',
      'plant->kq',
    ])
    const laneY = returns.map((wire) => wire.points.find((point) => point.y > sumE.y + sumE.h)?.y)
    expect(laneY[0]).toBeGreaterThan(sumE.y + sumE.h)
    expect(laneY[0]! - laneY[1]!).toBeGreaterThanOrEqual(36)

    const kq = layout.boxes.find((box) => box.key === '0:kq')!
    const outerY = laneY[0]!
    expect(outerY).toBeGreaterThan(kq.y + kq.h)
    for (let i = 0; i < layout.wires.length; i += 1) {
      for (let j = i + 1; j < layout.wires.length; j += 1) {
        const left = layout.wires[i].points
        const right = layout.wires[j].points
        for (let a = 0; a < left.length - 1; a += 1) {
          for (let b = 0; b < right.length - 1; b += 1) {
            expect(segmentsCross(left[a], left[a + 1], right[b], right[b + 1])).toBe(false)
          }
        }
      }
    }

    expect(plant.x).toBeGreaterThan(
      Math.max(...layout.boxes.filter((box) => box.key !== 'plant').map((box) => box.x + box.w)),
    )
    assertSeparate(layout)

    for (const wire of layout.wires) {
      if (wire.signAt === null) continue
      expect(wireDistance(wire.signAt, wire.points)).toBeGreaterThanOrEqual(12)
      const dest = layout.boxes.find((box) => box.node.id === wire.to && box.node.kind === 'sum')
      if (dest === undefined) continue
      const radius = Math.min(dest.w, dest.h) / 2 - 1
      const clear = Math.hypot(
        wire.signAt.x - (dest.x + dest.w / 2),
        wire.signAt.y - (dest.y + dest.h / 2),
      )
      expect(clear).toBeGreaterThanOrEqual(radius + 8)
    }
  })

  it('marks the measurement on each feedback line, above the wire', () => {
    const layout = layoutDiagram(PITCH)
    const theta = layout.wires.find((wire) => wire.from === 'plant' && wire.to === 'sum_e')!
    const rate = layout.wires.find((wire) => wire.from === 'plant' && wire.to === 'kq')!
    expect(theta.signal).toBe('theta')
    expect(rate.signal).toBe('q')
    for (const wire of [theta, rate]) {
      expect(wire.signalAt).not.toBeNull()
      const lane = wire.points.find((point) => point.y > 200)
      expect(wire.signalAt!.y).toBeLessThan(lane!.y)
      expect(wireDistance(wire.signalAt!, wire.points)).toBeGreaterThanOrEqual(10)
    }
    const { container } = render(createElement(ControllerDiagram, { diagram: PITCH }))
    const labels = [...container.querySelectorAll('[data-signal] text')].map((node) => node.textContent)
    expect(labels).toEqual(expect.arrayContaining(['theta', 'q']))
  })

  it('marks the variable on the forward path, under the wire', () => {
    const layout = layoutDiagram(PITCH)
    const command = layout.wires.find((wire) => wire.from === 'theta_ref')!
    const error = layout.wires.find((wire) => wire.from === 'sum_e' && wire.to === 'pi')!
    const channel = layout.wires.find((wire) => wire.to === 'plant')!
    expect(command.signal).toBe('theta')
    expect(error.signal).toBe('e_theta')
    expect(channel.signal).toBe('pitch')
    for (const wire of [command, error, channel]) {
      expect(wire.signalAt).not.toBeNull()
      const lineY = wire.points[0].y
      expect(wire.signalAt!.y).toBeGreaterThan(lineY)
      expect(wireDistance(wire.signalAt!, wire.points)).toBeGreaterThanOrEqual(10)
    }
    const disturbance = layout.wires.find((wire) => wire.from === 'disturbance')!
    expect(disturbance.signalAt).toBeNull()
    const { container } = render(createElement(ControllerDiagram, { diagram: PITCH }))
    const labels = [...container.querySelectorAll('[data-signal] text')].map((node) => node.textContent)
    expect(labels).toEqual(expect.arrayContaining(['theta', 'e_theta', 'pitch']))
  })

  it('draws each sum sign on a chip large enough to read', () => {
    const { container } = render(createElement(ControllerDiagram, { diagram: PITCH }))
    const signs = container.querySelectorAll('[data-sign]')
    expect(signs.length).toBeGreaterThan(0)
    for (const sign of signs) {
      expect(sign.querySelector('text')?.getAttribute('font-size')).toBe('16')
      expect(sign.querySelector('rect')).not.toBeNull()
    }
  })

  it('gives two side inputs separate entries along the top of one summer', () => {
    const layout = layoutDiagram([
      {
        input: 'r',
        output: 'yaw',
        feedback: 'r',
        nodes: [
          node('kr', 'block', 'kr', '1'),
          node('k_beta', 'block', 'k_beta', '1'),
          node('k_betadot', 'block', 'k_betadot', '1'),
          node('sum_yaw', 'sum', 'sum'),
        ],
        wires: [
          { from: 'kr', to: 'sum_yaw', sign: '+' },
          { from: 'k_beta', to: 'sum_yaw', sign: '+' },
          { from: 'k_betadot', to: 'sum_yaw', sign: '+' },
        ],
      },
    ])
    const sum = layout.boxes.find((box) => box.key === '0:sum_yaw')!
    const kr = layout.boxes.find((box) => box.key === '0:kr')!
    const beta = layout.boxes.find((box) => box.key === '0:k_beta')!
    const dot = layout.boxes.find((box) => box.key === '0:k_betadot')!
    const fromKr = layout.wires.find((wire) => wire.from === 'kr' && wire.to === 'sum_yaw')!
    const fromBeta = layout.wires.find((wire) => wire.from === 'k_beta' && wire.to === 'sum_yaw')!
    const fromDot = layout.wires.find((wire) => wire.from === 'k_betadot' && wire.to === 'sum_yaw')!
    const mid = kr.y + kr.h / 2
    expect(kr.y).toBe(sum.y)
    expect(fromKr.points.every((point) => point.y === mid)).toBe(true)
    expect(beta.y + beta.h).toBeLessThanOrEqual(sum.y - 24)
    expect(dot.y + dot.h).toBeLessThanOrEqual(sum.y - 24)
    expect(hitsBox(fromKr.points, beta)).toBe(false)
    expect(hitsBox(fromBeta.points, kr)).toBe(false)
    const betaEnd = fromBeta.points[fromBeta.points.length - 1]
    const dotEnd = fromDot.points[fromDot.points.length - 1]
    expect(betaEnd.y).toBeLessThanOrEqual(sum.y + 1)
    expect(dotEnd.y).toBeLessThanOrEqual(sum.y + 1)
    expect(betaEnd.x).not.toBe(dotEnd.x)
  })

  it('keeps parallel returns from crossing and puts the outer one lowest', () => {
    const layout = layoutDiagram([
      {
        input: 'r',
        output: 'yaw',
        feedback: 'r',
        nodes: [
          node('r_cmd', 'input', 'r_cmd'),
          node('sum_r', 'sum', 'sum'),
          node('kr_track', 'block', 'kr_track', '1'),
          node('k_beta', 'block', 'k_beta', '1'),
          node('k_betadot', 'block', 'k_betadot', '1'),
          node('sum_yaw', 'sum', 'sum'),
          node('plant', 'plant', 'airframe'),
        ],
        wires: [
          { from: 'r_cmd', to: 'sum_r', sign: '+' },
          { from: 'plant', to: 'sum_r', sign: '-' },
          { from: 'sum_r', to: 'kr_track', sign: '' },
          { from: 'kr_track', to: 'sum_yaw', sign: '+' },
          { from: 'plant', to: 'k_beta', sign: '' },
          { from: 'k_beta', to: 'sum_yaw', sign: '+' },
          { from: 'plant', to: 'k_betadot', sign: '' },
          { from: 'k_betadot', to: 'sum_yaw', sign: '+' },
          { from: 'sum_yaw', to: 'plant', sign: '' },
        ],
      },
    ])
    const yOf = (from: string, to: string) => {
      const wire = layout.wires.find((item) => item.from === from && item.to === to)!
      return Math.max(...wire.points.map((point) => point.y))
    }
    expect(yOf('plant', 'sum_r')).toBeGreaterThan(yOf('k_beta', 'sum_yaw'))
    expect(yOf('plant', 'sum_r')).toBeGreaterThan(yOf('k_betadot', 'sum_yaw'))
    for (let i = 0; i < layout.wires.length; i += 1) {
      for (let j = i + 1; j < layout.wires.length; j += 1) {
        const left = layout.wires[i].points
        const right = layout.wires[j].points
        for (let a = 0; a < left.length - 1; a += 1) {
          for (let b = 0; b < right.length - 1; b += 1) {
            expect(segmentsCross(left[a], left[a + 1], right[b], right[b + 1])).toBe(false)
          }
        }
      }
    }
  })

  it('keeps every wire on its own path and every block clear of the others', () => {
    const damper: DiagramLoop = {
      input: 'q',
      output: 'pitch',
      feedback: 'q',
      nodes: [
        node('kq', 'block', 'kq', '0.5'),
        node('sum_out', 'sum', 'sum'),
        node('disturbance', 'block', 'disturbance', 'doublet ±2 deg'),
        node('plant', 'plant', 'airframe'),
      ],
      wires: [
        { from: 'plant', to: 'kq', sign: '', signal: 'q' },
        { from: 'kq', to: 'sum_out', sign: '-' },
        { from: 'disturbance', to: 'sum_out', sign: '+' },
        { from: 'sum_out', to: 'plant', sign: '', signal: 'pitch' },
      ],
    }
    const speed: DiagramLoop = {
      input: 'vt',
      output: 'throttle',
      feedback: 'vt',
      nodes: [
        node('vt_ref', 'input', 'vt_ref'),
        node('sum_e', 'sum', 'sum'),
        node('pi', 'block', 'PI', '0.05 + 0.005/s'),
        node('plant', 'plant', 'airframe'),
      ],
      wires: [
        { from: 'vt_ref', to: 'sum_e', sign: '+', signal: 'vt' },
        { from: 'plant', to: 'sum_e', sign: '-', signal: 'vt' },
        { from: 'sum_e', to: 'pi', sign: '', signal: 'e_vt' },
        { from: 'pi', to: 'plant', sign: '', signal: 'throttle' },
      ],
    }
    const yaw: DiagramLoop = {
      input: 'r',
      output: 'yaw',
      feedback: 'r',
      nodes: [
        node('washout', 'block', 'washout', 's/(s+1)'),
        node('kr', 'block', 'kr', '0.5'),
        node('sum_out', 'sum', 'sum'),
        node('k_beta', 'block', 'k_beta', '1'),
        node('roll', 'input', 'roll'),
        node('k_ari', 'block', 'k_ari', '0.25'),
        node('plant', 'plant', 'airframe'),
      ],
      wires: [
        { from: 'plant', to: 'washout', sign: '', signal: 'r' },
        { from: 'washout', to: 'kr', sign: '' },
        { from: 'kr', to: 'sum_out', sign: '-' },
        { from: 'plant', to: 'k_beta', sign: '', signal: 'beta' },
        { from: 'k_beta', to: 'sum_out', sign: '+' },
        { from: 'roll', to: 'k_ari', sign: '' },
        { from: 'k_ari', to: 'sum_out', sign: '+' },
        { from: 'sum_out', to: 'plant', sign: '', signal: 'yaw' },
      ],
    }
    const laid = layoutDiagram([damper])
    const disturbance = laid.boxes.find((box) => box.node.id === 'disturbance')!
    const sum = laid.boxes.find((box) => box.node.id === 'sum_out')!
    expect(disturbance.y + disturbance.h).toBeLessThanOrEqual(sum.y)
    assertSeparate(laid)
    assertSeparate(layoutDiagram(PITCH))
    assertSeparate(layoutDiagram([PITCH[0], speed]))
    assertSeparate(layoutDiagram([PITCH[0], yaw]))
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
    expect(plant.x).toBeGreaterThan(
      Math.max(...layout.boxes.filter((box) => box.key !== 'plant').map((box) => box.x + box.w)),
    )
    assertSeparate(layout)
    const lane = wire.points.find((point) => point.y > lower.y + lower.h)!
    expect(lane.y).toBeGreaterThan(lower.y + lower.h)
    for (const box of upper) {
      if (box.key === 'plant') continue
      expect(lane.y).toBeGreaterThan(box.y + box.h)
    }
  })
})
