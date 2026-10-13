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
  signal: string
  feedback: boolean
  points: { x: number; y: number }[]
  signAt: { x: number; y: number } | null
  signalAt: { x: number; y: number } | null
}

export interface DiagramLayout {
  width: number
  height: number
  boxes: DiagramBox[]
  wires: RoutedWire[]
}

const PAD = 24
const GAP_X = 88
const BOX_H = 64
const ABOVE_GAP = 48
const FIRST_LANE = 40
const PLAIN_PITCH = 40
const BLOCK_CLEAR = 28
const ELBOW = 16
const SIGN_OFF = 22

type Role = 'spine' | 'above' | 'below'

interface Chain {
  blocks: string[]
  dest: string
  sign: '' | '+' | '-'
}

interface Lane {
  key: string
  dest: string
  sign: '' | '+' | '-'
  blocks: string[]
  y: number
  riserX: number
}

function nodeWidth(node: DiagramNode): number {
  const chars = Math.max(node.label.length, node.text.length, 4)
  return Math.max(128, chars * 8 + 24)
}

function boxKey(loopIndex: number, id: string): string {
  return id === 'plant' ? 'plant' : `${loopIndex}:${id}`
}

function isFeedback(_loop: DiagramLoop, wire: DiagramWire): boolean {
  // A measurement leaves the airframe. Every other wire, including a disturbance
  // or a cross-feed listed after its summer, runs forward.
  return wire.from === 'plant'
}

function incoming(loop: DiagramLoop, id: string): DiagramWire[] {
  return loop.wires.filter((wire) => wire.to === id && !isFeedback(loop, wire))
}

function reaches(loop: DiagramLoop, id: string, target: string, seen = new Set<string>()): boolean {
  if (id === target) return true
  if (seen.has(id)) return false
  seen.add(id)
  return incoming(loop, id).some((wire) => reaches(loop, wire.from, target, seen))
}

function primaryInput(loop: DiagramLoop): string | undefined {
  return loop.nodes.find((node) => node.kind === 'input')?.id
}

function choosePred(loop: DiagramLoop, wires: DiagramWire[]): string {
  const primary = primaryInput(loop)
  if (primary !== undefined) {
    const hit = wires.find((wire) => wire.from === primary || reaches(loop, wire.from, primary))
    if (hit !== undefined) return hit.from
  }
  const chained = wires.find((wire) => incoming(loop, wire.from).length > 0)
  return (chained ?? wires[0]).from
}

/** Forward path from the command (or the damper series) to the node that drives the plant. */
function spineIds(loop: DiagramLoop): string[] {
  const intoPlant = loop.wires.find((wire) => wire.to === 'plant' && !isFeedback(loop, wire))
  let cursor = intoPlant?.from
  if (cursor === undefined) {
    cursor = [...loop.nodes].reverse().find((node) => node.id !== 'plant')?.id
  }
  const chain: string[] = []
  const seen = new Set<string>()
  while (cursor !== undefined && !seen.has(cursor)) {
    seen.add(cursor)
    chain.push(cursor)
    const wires = incoming(loop, cursor)
    if (wires.length === 0) break
    cursor = choosePred(loop, wires)
  }
  chain.reverse()
  return chain
}

function fedFromPlant(loop: DiagramLoop, id: string, seen = new Set<string>()): boolean {
  if (id === 'plant') return true
  if (seen.has(id)) return false
  seen.add(id)
  return loop.wires
    .filter((wire) => wire.to === id)
    .some((wire) => fedFromPlant(loop, wire.from, seen))
}

function chainsFrom(loop: DiagramLoop, pool: Set<string>, spine: Set<string>): Chain[] {
  const sources = [...pool].filter(
    (id) => !loop.wires.some((wire) => wire.to === id && pool.has(wire.from)),
  )
  const chains: Chain[] = []
  for (const source of sources) {
    const blocks: string[] = []
    let cursor = source
    let sign: '' | '+' | '-' = ''
    const seen = new Set<string>()
    while (pool.has(cursor) && !seen.has(cursor)) {
      seen.add(cursor)
      blocks.push(cursor)
      const next = loop.wires.find((wire) => wire.from === cursor && !isFeedback(loop, wire))
      if (next === undefined) break
      sign = next.sign
      if (spine.has(next.to)) {
        chains.push({ blocks, dest: next.to, sign })
        break
      }
      cursor = next.to
    }
  }
  return chains
}

function cx(box: DiagramBox): number {
  return box.x + box.w / 2
}

function cy(box: DiagramBox): number {
  return box.y + box.h / 2
}

function entryXs(dest: DiagramBox, count: number): number[] {
  if (count <= 1) return [cx(dest)]
  const inset = 14
  const left = dest.x + inset
  const right = dest.x + dest.w - inset
  if (right <= left) return Array.from({ length: count }, () => cx(dest))
  return Array.from({ length: count }, (_, index) => left + ((right - left) * index) / (count - 1))
}

function lanePitch(hasBlock: boolean, nextHasBlock: boolean): number {
  if (!hasBlock && !nextHasBlock) return PLAIN_PITCH
  if (!hasBlock && nextHasBlock) return BLOCK_CLEAR + BOX_H / 2
  if (hasBlock && !nextHasBlock) return BOX_H / 2 + BLOCK_CLEAR
  return BOX_H + BLOCK_CLEAR
}

interface RowDraft {
  loopIndex: number
  loop: DiagramLoop
  spine: string[]
  boxes: Map<string, DiagramBox>
  roles: Map<string, Role>
  lanes: Lane[]
  above: Chain[]
  spineRight: number
}

function draftRow(loop: DiagramLoop, loopIndex: number): RowDraft | null {
  const spine = spineIds(loop)
  if (spine.length === 0 && !loop.nodes.some((node) => node.id !== 'plant')) return null
  const spineSet = new Set(spine)
  const boxes = new Map<string, DiagramBox>()
  const roles = new Map<string, Role>()
  for (const node of loop.nodes) {
    if (node.id === 'plant') continue
    const box: DiagramBox = {
      key: boxKey(loopIndex, node.id),
      node,
      x: 0,
      y: 0,
      w: node.kind === 'sum' ? BOX_H : nodeWidth(node),
      h: BOX_H,
    }
    boxes.set(node.id, box)
    roles.set(node.id, spineSet.has(node.id) ? 'spine' : 'below')
  }
  const off = [...roles.keys()].filter((id) => !spineSet.has(id))
  const aboveSet = new Set(off.filter((id) => !fedFromPlant(loop, id)))
  const belowSet = new Set(off.filter((id) => fedFromPlant(loop, id)))
  for (const id of aboveSet) roles.set(id, 'above')

  let x = PAD
  for (const id of spine) {
    const box = boxes.get(id)
    if (box === undefined) continue
    box.x = x
    box.y = 0
    x += box.w + GAP_X
  }
  const spineRight = spine.length === 0 ? PAD : x - GAP_X

  const above = chainsFrom(loop, aboveSet, spineSet)
  const byDest = new Map<string, Chain[]>()
  for (const chain of above) {
    const group = byDest.get(chain.dest) ?? []
    group.push(chain)
    byDest.set(chain.dest, group)
  }
  for (const [dest, group] of byDest) {
    const destBox = boxes.get(dest)
    if (destBox === undefined) continue
    const widths = group.map((chain) =>
      chain.blocks.reduce(
        (sum, id, index) => sum + (boxes.get(id)?.w ?? 0) + (index === 0 ? 0 : GAP_X),
        0,
      ),
    )
    const total = widths.reduce((sum, width) => sum + width, 0) + Math.max(0, group.length - 1) * GAP_X
    let cursor = cx(destBox) - total / 2
    const y = -(ABOVE_GAP + BOX_H)
    for (const chain of group) {
      for (const id of chain.blocks) {
        const box = boxes.get(id)
        if (box === undefined) continue
        box.x = cursor
        box.y = y
        cursor += box.w + GAP_X
      }
    }
  }

  const lanes: Lane[] = []
  const consumed = new Set<string>()
  for (const wire of loop.wires) {
    if (!isFeedback(loop, wire)) continue
    if (spineSet.has(wire.to)) {
      lanes.push({ key: `${wire.from}->${wire.to}`, dest: wire.to, sign: wire.sign, blocks: [], y: 0, riserX: 0 })
      continue
    }
    if (!belowSet.has(wire.to) || consumed.has(wire.to)) continue
    const chain = chainsFrom(loop, belowSet, spineSet).find((item) => item.blocks[0] === wire.to)
    if (chain === undefined) continue
    for (const id of chain.blocks) consumed.add(id)
    lanes.push({
      key: `${wire.from}->${wire.to}`,
      dest: chain.dest,
      sign: chain.sign,
      blocks: chain.blocks,
      y: 0,
      riserX: 0,
    })
  }
  // A summer later on the spine closes the inner loop. Its return stays above
  // the outer return so the longer wire runs underneath and does not cross it.
  lanes.sort((a, b) => spine.indexOf(b.dest) - spine.indexOf(a.dest))
  let laneY = BOX_H + FIRST_LANE
  if (lanes[0] !== undefined && lanes[0].blocks.length > 0) laneY += BOX_H / 2
  lanes.forEach((lane, index) => {
    lane.y = laneY
    const next = lanes[index + 1]
    if (next !== undefined) laneY += lanePitch(lane.blocks.length > 0, next.blocks.length > 0)
  })

  return { loopIndex, loop, spine, boxes, roles, lanes, above, spineRight }
}

function shiftDraft(row: RowDraft, dx: number) {
  if (dx === 0) return
  for (const box of row.boxes.values()) box.x += dx
  row.spineRight += dx
}

function placeBelow(row: RowDraft, slotLeft: number) {
  for (const lane of row.lanes) {
    const dest = row.boxes.get(lane.dest)
    const sharing = row.lanes.filter((item) => item.dest === lane.dest && item.blocks.length > 0)
    const index = Math.max(0, sharing.indexOf(lane))
    // The upper return takes the right-hand entry. A lower return then rises
    // to the left of that wire instead of cutting through it.
    const xs = dest === undefined ? [slotLeft] : entryXs(dest, Math.max(1, sharing.length))
    lane.riserX = xs[xs.length - 1 - index] ?? slotLeft
    if (lane.blocks.length === 0) {
      if (dest !== undefined) lane.riserX = cx(dest)
      continue
    }
    const ordered = [...lane.blocks].reverse()
    const width = ordered.reduce((sum, id, index) => {
      const box = row.boxes.get(id)
      return sum + (box?.w ?? 0) + (index === 0 ? 0 : GAP_X)
    }, 0)
    const tucked = row.spineRight - width
    const clearOfRiser = lane.riserX + 20
    let x = tucked >= clearOfRiser ? tucked : Math.max(slotLeft, clearOfRiser)
    for (const id of ordered) {
      const box = row.boxes.get(id)
      if (box === undefined) continue
      box.x = x
      box.y = lane.y - BOX_H / 2
      x += box.w + GAP_X
    }
  }
  for (const lane of row.lanes) {
    if (lane.blocks.length > 0) continue
    const dest = row.boxes.get(lane.dest)
    if (dest !== undefined) lane.riserX = cx(dest)
  }
}

function relativeInk(row: RowDraft): { top: number; bottom: number } {
  let top = 0
  let bottom = BOX_H
  for (const box of row.boxes.values()) {
    top = Math.min(top, box.y)
    bottom = Math.max(bottom, box.y + box.h)
  }
  for (const lane of row.lanes) bottom = Math.max(bottom, lane.y)
  return { top, bottom }
}

function straight(from: DiagramBox, to: DiagramBox) {
  const y = cy(from)
  return [
    { x: from.x + from.w, y },
    { x: to.x, y },
  ]
}

function toPlant(from: DiagramBox, plant: DiagramBox) {
  const y = cy(from)
  return [
    { x: from.x + from.w, y },
    { x: plant.x, y },
  ]
}

function edgeSlots(plant: DiagramBox, count: number): number[] {
  if (count === 0) return []
  const left = plant.x + 16
  const right = plant.x + plant.w - 16
  if (count === 1) return [(left + right) / 2]
  return Array.from({ length: count }, (_, index) => left + ((right - left) * index) / (count - 1))
}

function feedbackLaneY(row: RowDraft, wire: DiagramWire): number | null {
  const to = row.boxes.get(wire.to)
  if (to === undefined) return null
  const role = row.roles.get(wire.to)
  if (role === 'spine') {
    const lane = row.lanes.find((item) => item.key === `${wire.from}->${wire.to}`)
    return lane?.y ?? to.y + to.h + FIRST_LANE
  }
  if (role === 'below') {
    const lane = row.lanes.find((item) => item.blocks[0] === wire.to)
    return lane?.y ?? cy(to)
  }
  return null
}

/** Leave the airframe on this wire's own port, never along a line another wire uses. */
function fromPlant(
  plant: DiagramBox,
  laneY: number,
  xEnd: number,
  slotX: number | undefined,
): { x: number; y: number }[] {
  const onFace = laneY >= plant.y && laneY <= plant.y + plant.h
  if (onFace || slotX === undefined) {
    return [
      { x: plant.x, y: laneY },
      { x: xEnd, y: laneY },
    ]
  }
  const yEdge = laneY > plant.y + plant.h ? plant.y + plant.h : plant.y
  return [
    { x: slotX, y: yEdge },
    { x: slotX, y: laneY },
    { x: xEnd, y: laneY },
  ]
}

function dedupe(points: { x: number; y: number }[]): { x: number; y: number }[] {
  const kept = [points[0]]
  for (const point of points.slice(1)) {
    const prev = kept[kept.length - 1]
    if (Math.abs(prev.x - point.x) > 0.5 || Math.abs(prev.y - point.y) > 0.5) kept.push(point)
  }
  return kept
}

function aboveDrop(from: DiagramBox, dest: DiagramBox, entryX: number) {
  const x0 = cx(from)
  const y0 = from.y + from.h
  if (Math.abs(x0 - entryX) < 0.5) {
    return [
      { x: x0, y: y0 },
      { x: entryX, y: dest.y },
    ]
  }
  const jog = dest.y - 36
  return [
    { x: x0, y: y0 },
    { x: x0, y: jog },
    { x: entryX, y: jog },
    { x: entryX, y: dest.y },
  ]
}


function signAtFor(
  sign: '' | '+' | '-',
  where: 'left' | 'top' | 'bottom',
  dest: DiagramBox,
  entryX: number,
): { x: number; y: number } | null {
  if (sign === '') return null
  if (where === 'top') return { x: entryX + SIGN_OFF, y: dest.y - 20 }
  if (where === 'bottom') return { x: entryX + SIGN_OFF, y: dest.y + dest.h + 16 }
  return { x: dest.x - SIGN_OFF, y: cy(dest) - 24 }
}

export function signalChip(text: string): { w: number; h: number } {
  return { w: Math.max(18, text.length * 7 + 8), h: 18 }
}

function chipHits(x: number, y: number, w: number, h: number, boxes: DiagramBox[]): boolean {
  const left = x - w / 2
  const top = y - h / 2
  return boxes.some(
    (box) => left < box.x + box.w && left + w > box.x && top < box.y + box.h && top + h > box.y,
  )
}

/** A name beside the longest horizontal run, clear of the blocks. */
function placeSignal(
  points: { x: number; y: number }[],
  text: string,
  boxes: DiagramBox[],
  side: 'above' | 'below',
): { x: number; y: number } | null {
  if (text === '') return null
  let best: { left: number; right: number; y: number } | null = null
  for (let index = 0; index < points.length - 1; index += 1) {
    const a = points[index]
    const b = points[index + 1]
    if (a.y !== b.y) continue
    const left = Math.min(a.x, b.x)
    const right = Math.max(a.x, b.x)
    if (best === null || right - left > best.right - best.left) best = { left, right, y: a.y }
  }
  if (best === null) return null
  const { w, h } = signalChip(text)
  const mid = (best.left + best.right) / 2
  const span = best.right - best.left
  const directions = side === 'above' ? [-1, 1] : [1, -1]
  for (const direction of directions) {
    for (let lift = 16; lift <= 40; lift += 8) {
      const y = best.y + direction * lift
      const room = Math.max(span, w)
      for (let offset = 0; offset <= room; offset += 8) {
        for (const x of offset === 0 ? [mid] : [mid - offset, mid + offset]) {
          if (span >= w && (x - w / 2 < best.left || x + w / 2 > best.right)) continue
          if (!chipHits(x, y, w, h, boxes)) return { x, y }
        }
      }
    }
  }
  return { x: mid, y: best.y + (side === 'above' ? -16 : 16) }
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
    if (wire.signAt !== null) grow(wire.signAt.x + 10, wire.signAt.y + 12)
    if (wire.signalAt !== null) {
      const chip = signalChip(wire.signal)
      grow(wire.signalAt.x + chip.w / 2, wire.signalAt.y + chip.h / 2)
    }
  }
  if (boxes.length === 0 && wires.length === 0) return { width: 0, height: 0 }
  return { width: width + PAD, height: height + PAD }
}

export function layoutDiagram(diagram: DiagramLoop[]): DiagramLayout {
  const rows = diagram
    .map((loop, loopIndex) => draftRow(loop, loopIndex))
    .filter((row): row is RowDraft => row !== null)

  for (const row of rows) {
    let minX = PAD
    for (const [id, box] of row.boxes) {
      if (row.roles.get(id) === 'below') continue
      minX = Math.min(minX, box.x)
    }
    shiftDraft(row, PAD - minX)
  }

  let slotLeft = PAD
  for (const row of rows) slotLeft = Math.max(slotLeft, row.spineRight + 16)
  for (const row of rows) placeBelow(row, slotLeft)

  const bases: number[] = []
  let cursor = PAD
  for (const row of rows) {
    const ink = relativeInk(row)
    const base = cursor - ink.top
    bases.push(base)
    for (const box of row.boxes.values()) box.y += base
    for (const lane of row.lanes) lane.y += base
    cursor = base + ink.bottom + 28
  }

  const boxes: DiagramBox[] = rows.flatMap((row) => [...row.boxes.values()])
  const plantNode = diagram.flatMap((loop) => loop.nodes).find((node) => node.id === 'plant')
  let plant: DiagramBox | null = null
  if (plantNode !== undefined) {
    const rightmost = boxes.reduce((max, box) => Math.max(max, box.x + box.w), PAD)
    const spineBoxes = rows.flatMap((row) =>
      row.spine
        .map((id) => row.boxes.get(id))
        .filter((box): box is DiagramBox => box !== undefined),
    )
    const top = spineBoxes.length === 0 ? PAD : Math.min(...spineBoxes.map((box) => box.y))
    const bottom = spineBoxes.length === 0 ? top + BOX_H : Math.max(...spineBoxes.map((box) => box.y + box.h))
    plant = {
      key: 'plant',
      node: plantNode,
      x: rightmost + ELBOW + 12,
      y: top,
      w: nodeWidth(plantNode),
      h: Math.max(BOX_H, bottom - top),
    }
    boxes.push(plant)
  }

  const slotOf = new Map<string, number>()
  if (plant !== null) {
    const exits: { key: string; laneY: number }[] = []
    rows.forEach((row, rowIndex) => {
      for (const wire of row.loop.wires) {
        if (wire.from !== 'plant') continue
        const laneY = feedbackLaneY(row, wire)
        if (laneY === null) continue
        if (laneY < plant.y || laneY > plant.y + plant.h) {
          exits.push({ key: `${rowIndex}:${wire.to}`, laneY })
        }
      }
    })
    exits.sort((a, b) => a.laneY - b.laneY)
    const slots = edgeSlots(plant, exits.length)
    exits.forEach((exit, index) => slotOf.set(exit.key, slots[index] ?? plant.x + plant.w / 2))
  }

  const wires: RoutedWire[] = []
  rows.forEach((row, rowIndex) => {
    const aboveEntry = new Map<string, number>()
    const grouped = new Map<string, Chain[]>()
    for (const chain of row.above) {
      const group = grouped.get(chain.dest) ?? []
      group.push(chain)
      grouped.set(chain.dest, group)
    }
    for (const [dest, group] of grouped) {
      const destBox = row.boxes.get(dest)
      if (destBox === undefined) continue
      const xs = entryXs(destBox, group.length)
      group.forEach((chain, index) => {
        const last = chain.blocks[chain.blocks.length - 1]
        if (last !== undefined) aboveEntry.set(`${last}->${dest}`, xs[index] ?? cx(destBox))
      })
    }

    const spineIndex = new Map(row.spine.map((id, index) => [id, index]))
    for (const wire of row.loop.wires) {
      const feedback = isFeedback(row.loop, wire)
      const from = wire.from === 'plant' ? plant : row.boxes.get(wire.from)
      const to = wire.to === 'plant' ? plant : row.boxes.get(wire.to)
      if (from === null || to === null || from === undefined || to === undefined) continue
      const fromRole = row.roles.get(wire.from)
      const toRole = row.roles.get(wire.to)
      let points: { x: number; y: number }[]
      let where: 'left' | 'top' | 'bottom' = 'left'
      let entryX = to.x

      const fromSpine = spineIndex.get(wire.from)
      const toSpine = spineIndex.get(wire.to)
      if (
        fromSpine !== undefined &&
        toSpine !== undefined &&
        toSpine === fromSpine + 1
      ) {
        points = straight(from, to)
      } else if (wire.to === 'plant' && plant !== null) {
        points = toPlant(from, plant)
      } else if (feedback && wire.from === 'plant' && plant !== null) {
        const laneY = feedbackLaneY(row, wire) ?? cy(to)
        const slotX = slotOf.get(`${rowIndex}:${wire.to}`)
        if (toRole === 'spine') {
          const lane = row.lanes.find((item) => item.key === `${wire.from}->${wire.to}`)
          const riser = lane?.riserX ?? cx(to)
          points = [...fromPlant(plant, laneY, riser, slotX), { x: riser, y: to.y + to.h }]
          where = 'bottom'
          entryX = riser
        } else {
          points = fromPlant(plant, laneY, to.x + to.w, slotX)
        }
      } else if (fromRole === 'below' && toRole === 'below') {
        points = [
          { x: from.x, y: cy(from) },
          { x: to.x + to.w, y: cy(to) },
        ]
      } else if (fromRole === 'below' && toRole === 'spine') {
        const lane = row.lanes.find((item) => item.blocks[item.blocks.length - 1] === wire.from)
        const riser = lane?.riserX ?? cx(to)
        const y = cy(from)
        points = [
          { x: from.x, y },
          { x: riser, y },
          { x: riser, y: to.y + to.h },
        ]
        where = 'bottom'
        entryX = riser
      } else if (fromRole === 'above' && toRole === 'above') {
        points = straight(from, to)
      } else if (fromRole === 'above' && toRole === 'spine') {
        entryX = aboveEntry.get(`${wire.from}->${wire.to}`) ?? cx(to)
        points = aboveDrop(from, to, entryX)
        where = 'top'
      } else if (feedback) {
        const y = to.y + to.h + FIRST_LANE
        points = [
          { x: cx(from), y: from.y + from.h },
          { x: cx(from), y },
          { x: cx(to), y },
          { x: cx(to), y: to.y + to.h },
        ]
        where = 'bottom'
        entryX = cx(to)
      } else {
        points = straight(from, to)
      }

      const signal = wire.signal ?? ''
      wires.push({
        from: wire.from,
        to: wire.to,
        sign: wire.sign,
        signal,
        feedback,
        points: dedupe(points),
        signAt: signAtFor(wire.sign, where, to, entryX),
        signalAt: signal === '' ? null : placeSignal(points, signal, boxes, feedback ? 'above' : 'below'),
      })
    }
  })

  return { ...extent(boxes, wires), boxes, wires }
}
