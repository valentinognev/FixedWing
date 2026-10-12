import type { DiagramLoop, DiagramNode, DiagramWire } from './types'

export interface DiagramBox {
  key: string
  node: DiagramNode
  x: number
  y: number
  w: number
  h: number
}

export interface RoutedWire {
  from: string
  to: string
  sign: '' | '+' | '-'
  feedback: boolean
  points: { x: number; y: number }[]
  signAt: { x: number; y: number } | null
}

export interface DiagramLayout {
  width: number
  height: number
  boxes: DiagramBox[]
  wires: RoutedWire[]
}

const PAD = 20
const GAP_X = 36
const BOX_H = 64

function nodeWidth(node: DiagramNode): number {
  const chars = Math.max(node.label.length, node.text.length, 4)
  return Math.max(128, chars * 8 + 24)
}

/** Plant is one box for every loop. Every other id is local to its loop. */
function boxKey(loopIndex: number, id: string): string {
  return id === 'plant' ? 'plant' : `${loopIndex}:${id}`
}

/** A return runs backward through the loop, or leaves the shared airframe. */
function isFeedback(loop: DiagramLoop, wire: DiagramWire): boolean {
  if (wire.from === 'plant') return true
  const fromIndex = loop.nodes.findIndex((node) => node.id === wire.from)
  const toIndex = loop.nodes.findIndex((node) => node.id === wire.to)
  return fromIndex > toIndex
}

function rowStep(feedbackCount: number): number {
  if (feedbackCount === 0) return BOX_H + 28
  return BOX_H + 12 + feedbackCount * 16 + 12
}

function cx(box: DiagramBox): number {
  return box.x + box.w / 2
}

function cy(box: DiagramBox): number {
  return box.y + box.h / 2
}

function bottom(box: DiagramBox): number {
  return box.y + box.h
}

function clamp(value: number, low: number, high: number): number {
  return Math.min(high, Math.max(low, value))
}

function laneY(rowTop: number, lane: number): number {
  return rowTop + BOX_H + 12 + lane * 16
}

function plantReturn(plant: DiagramBox, dest: DiagramBox, rowTop: number, lane: number) {
  const yOnPlant = clamp(rowTop + BOX_H / 2, plant.y, plant.y + plant.h)
  const y = laneY(rowTop, lane)
  const destX = cx(dest)
  return [
    { x: plant.x, y: yOnPlant },
    { x: plant.x - 14, y: yOnPlant },
    { x: plant.x - 14, y },
    { x: destX, y },
    { x: destX, y: bottom(dest) },
  ]
}

function sameRowReturn(from: DiagramBox, dest: DiagramBox, rowTop: number, lane: number) {
  const y = laneY(rowTop, lane)
  return [
    { x: cx(from), y: bottom(from) },
    { x: cx(from), y },
    { x: cx(dest), y },
    { x: cx(dest), y: bottom(dest) },
  ]
}

function forwardSameRow(from: DiagramBox, dest: DiagramBox) {
  return [
    { x: from.x + from.w, y: cy(from) },
    { x: dest.x, y: cy(dest) },
  ]
}

function forwardIntoPlant(from: DiagramBox, plant: DiagramBox) {
  return [
    { x: from.x + from.w, y: cy(from) },
    { x: plant.x - 14, y: cy(from) },
    { x: plant.x - 14, y: cy(plant) },
    { x: plant.x, y: cy(plant) },
  ]
}

function route(
  wire: DiagramWire,
  from: DiagramBox,
  to: DiagramBox,
  feedback: boolean,
  lane: number,
  rowTop: number,
): RoutedWire {
  let points
  if (feedback && wire.from === 'plant') points = plantReturn(from, to, rowTop, lane)
  else if (feedback) points = sameRowReturn(from, to, rowTop, lane)
  else if (wire.to === 'plant') points = forwardIntoPlant(from, to)
  else points = forwardSameRow(from, to)

  const signAt =
    wire.sign === ''
      ? null
      : feedback
        ? { x: cx(to) + 14, y: bottom(to) + 11 }
        : { x: to.x - 12, y: cy(to) }

  return {
    from: wire.from,
    to: wire.to,
    sign: wire.sign,
    feedback,
    points,
    signAt,
  }
}

function extent(boxes: DiagramBox[], wires: RoutedWire[]): { width: number; height: number } {
  let width = 0
  let height = 0
  const grow = (x: number, y: number) => {
    width = Math.max(width, x)
    height = Math.max(height, y)
  }
  for (const box of boxes) grow(box.x + box.w, box.y + box.h)
  for (const wire of wires) {
    for (const point of wire.points) grow(point.x, point.y)
    if (wire.signAt !== null) grow(wire.signAt.x, wire.signAt.y)
  }
  if (boxes.length === 0 && wires.length === 0) return { width: 0, height: 0 }
  return { width: width + PAD, height: height + PAD }
}

export function layoutDiagram(diagram: DiagramLoop[]): DiagramLayout {
  const boxes: DiagramBox[] = []
  const rowTop = new Map<number, number>()
  let y = PAD
  let maxRight = PAD
  let rows = 0
  let lastBottom = PAD

  diagram.forEach((loop, loopIndex) => {
    let x = PAD
    const seen = new Set<string>()
    let placed = false
    for (const node of loop.nodes) {
      if (node.id === 'plant' || seen.has(node.id)) continue
      seen.add(node.id)
      const w = node.kind === 'sum' ? BOX_H : nodeWidth(node)
      boxes.push({ key: boxKey(loopIndex, node.id), node, x, y, w, h: BOX_H })
      x += w + GAP_X
      placed = true
    }
    if (!placed) return
    rowTop.set(loopIndex, y)
    rows += 1
    maxRight = Math.max(maxRight, x - GAP_X)
    lastBottom = y + BOX_H
    y += rowStep(loop.wires.filter((wire) => isFeedback(loop, wire)).length)
  })

  const plantNode = diagram.flatMap((loop) => loop.nodes).find((node) => node.id === 'plant')
  if (plantNode !== undefined) {
    const stack = rows === 0 ? BOX_H : lastBottom - PAD
    boxes.push({
      key: 'plant',
      node: plantNode,
      x: rows === 0 ? PAD : maxRight + GAP_X,
      y: PAD + stack / 2 - BOX_H / 2,
      w: nodeWidth(plantNode),
      h: BOX_H,
    })
  }

  const byKey = new Map(boxes.map((box) => [box.key, box]))
  const wires: RoutedWire[] = []
  diagram.forEach((loop, loopIndex) => {
    const top = rowTop.get(loopIndex)
    let lane = 0
    for (const wire of loop.wires) {
      const feedback = isFeedback(loop, wire)
      const laneIndex = lane
      if (feedback) lane += 1
      const from = byKey.get(boxKey(loopIndex, wire.from))
      const to = byKey.get(boxKey(loopIndex, wire.to))
      if (from === undefined || to === undefined || top === undefined) continue
      wires.push(route(wire, from, to, feedback, laneIndex, top))
    }
  })

  return { ...extent(boxes, wires), boxes, wires }
}
