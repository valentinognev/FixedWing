import type { DiagramLoop, DiagramNode, DiagramWire } from '../types'

export interface ControllerDiagramProps {
  diagram: DiagramLoop[]
}

const PANEL = 'flex flex-col gap-4 rounded border border-neutral-800 bg-neutral-900/50 p-4'
const TITLE = 'text-xs font-semibold tracking-wide text-neutral-400 uppercase'

const PAD = 20
const GAP_X = 36
const ROW_GAP = 28
const BOX_H = 64

interface Box {
  node: DiagramNode
  x: number
  y: number
  w: number
  h: number
}

function nodeWidth(node: DiagramNode): number {
  const chars = Math.max(node.label.length, node.text.length, 4)
  return Math.max(128, chars * 8 + 24)
}

/** Plant is one box for every loop. Every other id is local to its loop. */
function boxKey(loopIndex: number, id: string): string {
  return id === 'plant' ? 'plant' : `${loopIndex}:${id}`
}

function layout(diagram: DiagramLoop[]): Map<string, Box> {
  const boxes = new Map<string, Box>()
  let y = PAD
  let maxRight = PAD
  let rows = 0

  diagram.forEach((loop, loopIndex) => {
    let x = PAD
    const seen = new Set<string>()
    let placed = false
    for (const node of loop.nodes) {
      if (node.id === 'plant' || seen.has(node.id)) continue
      seen.add(node.id)
      const w = node.kind === 'sum' ? BOX_H : nodeWidth(node)
      boxes.set(boxKey(loopIndex, node.id), { node, x, y, w, h: BOX_H })
      x += w + GAP_X
      placed = true
    }
    if (!placed) return
    rows += 1
    maxRight = Math.max(maxRight, x - GAP_X)
    y += BOX_H + ROW_GAP
  })

  const plant = diagram.flatMap((loop) => loop.nodes).find((node) => node.id === 'plant')
  if (plant !== undefined) {
    const stack = rows === 0 ? BOX_H : rows * BOX_H + Math.max(rows - 1, 0) * ROW_GAP
    boxes.set('plant', {
      node: plant,
      x: rows === 0 ? PAD : maxRight + GAP_X,
      y: PAD + stack / 2 - BOX_H / 2,
      w: nodeWidth(plant),
      h: BOX_H,
    })
  }
  return boxes
}

function center(box: Box): { x: number; y: number } {
  return { x: box.x + box.w / 2, y: box.y + box.h / 2 }
}

function signAt(from: Box, to: Box): { x: number; y: number } {
  const a = center(from)
  const b = center(to)
  const t = 0.62
  const x = a.x + (b.x - a.x) * t
  const y = a.y + (b.y - a.y) * t
  const dx = b.x - a.x
  const dy = b.y - a.y
  const length = Math.hypot(dx, dy) || 1
  return { x: x + (-dy / length) * 12, y: y + (dx / length) * 12 }
}

function NodeGlyph({ box }: { box: Box }) {
  const { node, x, y, w, h } = box
  if (node.kind === 'sum') {
    return (
      <circle
        cx={x + w / 2}
        cy={y + h / 2}
        r={Math.min(w, h) / 2 - 1}
        fill="#171717"
        stroke="#a3a3a3"
      />
    )
  }
  return (
    <rect
      x={x}
      y={y}
      width={w}
      height={h}
      rx={6}
      fill="#171717"
      stroke={node.kind === 'plant' ? '#0284c7' : '#525252'}
      strokeWidth={node.kind === 'plant' ? 1.5 : 1}
    />
  )
}

function NodeLabel({ box }: { box: Box }) {
  const { node, x, y, w, h } = box
  const cx = x + w / 2
  const mid = y + h / 2
  return (
    <>
      <text
        x={cx}
        y={node.text === '' ? mid : mid - 9}
        textAnchor="middle"
        dominantBaseline="middle"
        fill="#e5e5e5"
        fontSize="12"
      >
        {node.label}
      </text>
      {node.text !== '' && (
        <text
          x={cx}
          y={mid + 11}
          textAnchor="middle"
          dominantBaseline="middle"
          fill="#d4d4d4"
          fontSize="12"
        >
          {node.text}
        </text>
      )}
    </>
  )
}

function WireLine({ wire, from, to }: { wire: DiagramWire; from: Box; to: Box }) {
  const a = center(from)
  const b = center(to)
  const sign = wire.sign === '' ? null : signAt(from, to)
  return (
    <g>
      <line
        x1={a.x}
        y1={a.y}
        x2={b.x}
        y2={b.y}
        stroke="#737373"
        strokeWidth={1.25}
        markerEnd="url(#analysis-arrow)"
      />
      {sign !== null && (
        <text
          x={sign.x}
          y={sign.y}
          textAnchor="middle"
          dominantBaseline="middle"
          fill="#fbbf24"
          fontSize="13"
        >
          {wire.sign}
        </text>
      )}
    </g>
  )
}

/** One SVG. The plant id is on every loop and is drawn once; other ids stay per loop. */
export function ControllerDiagram({ diagram }: ControllerDiagramProps) {
  const boxes = layout(diagram)
  let width = 0
  let height = 0
  for (const box of boxes.values()) {
    width = Math.max(width, box.x + box.w)
    height = Math.max(height, box.y + box.h)
  }
  if (boxes.size > 0) {
    width += PAD
    height += PAD
  }

  return (
    <section className={PANEL}>
      <h2 className={TITLE}>Controller</h2>
      <div className="overflow-x-auto">
        <svg width={width} height={height}>
          <defs>
            <marker
              id="analysis-arrow"
              markerWidth="8"
              markerHeight="8"
              refX="7"
              refY="3"
              orient="auto"
            >
              <path d="M0,0 L7,3 L0,6" fill="#737373" />
            </marker>
          </defs>
          {diagram.map((loop, loopIndex) =>
            loop.wires.map((wire, wireIndex) => {
              const from = boxes.get(boxKey(loopIndex, wire.from))
              const to = boxes.get(boxKey(loopIndex, wire.to))
              if (from === undefined || to === undefined) return null
              return (
                <WireLine
                  key={`${loopIndex}:${wire.from}-${wire.to}-${wire.sign}-${wireIndex}`}
                  wire={wire}
                  from={from}
                  to={to}
                />
              )
            }),
          )}
          {[...boxes.entries()].map(([key, box]) => (
            <g key={key}>
              <NodeGlyph box={box} />
              <NodeLabel box={box} />
            </g>
          ))}
        </svg>
      </div>
    </section>
  )
}
