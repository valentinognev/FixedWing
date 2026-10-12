import { layoutDiagram, type DiagramBox } from '../diagramLayout'
import type { DiagramLoop } from '../types'

export interface ControllerDiagramProps {
  diagram: DiagramLoop[]
}

const PANEL = 'flex flex-col gap-4 rounded border border-neutral-800 bg-neutral-900/50 p-4'
const TITLE = 'text-xs font-semibold tracking-wide text-neutral-400 uppercase'

const FORWARD = '#737373'
const FEEDBACK = '#38bdf8'

function NodeGlyph({ box }: { box: DiagramBox }) {
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

function NodeLabel({ box }: { box: DiagramBox }) {
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

function Arrow({ id, fill }: { id: string; fill: string }) {
  return (
    <marker id={id} markerWidth="8" markerHeight="8" refX="7" refY="3" orient="auto">
      <path d="M0,0 L7,3 L0,6" fill={fill} />
    </marker>
  )
}

/** One SVG. The plant id is on every loop and is drawn once; other ids stay per loop. */
export function ControllerDiagram({ diagram }: ControllerDiagramProps) {
  const layout = layoutDiagram(diagram)

  return (
    <section className={PANEL}>
      <h2 className={TITLE}>Controller</h2>
      <div className="overflow-x-auto">
        <svg width={layout.width} height={layout.height}>
          <defs>
            <Arrow id="analysis-arrow" fill={FORWARD} />
            <Arrow id="analysis-arrow-feedback" fill={FEEDBACK} />
          </defs>
          {layout.wires.map((wire, index) => (
            <g key={`${wire.from}-${wire.to}-${wire.sign}-${index}`}>
              <polyline
                points={wire.points.map((point) => `${point.x},${point.y}`).join(' ')}
                fill="none"
                stroke={wire.feedback ? FEEDBACK : FORWARD}
                strokeWidth={wire.feedback ? 1.75 : 1.25}
                markerEnd={
                  wire.feedback ? 'url(#analysis-arrow-feedback)' : 'url(#analysis-arrow)'
                }
              />
              {wire.signAt !== null && (
                <text
                  x={wire.signAt.x}
                  y={wire.signAt.y}
                  textAnchor="middle"
                  dominantBaseline="middle"
                  fill="#fbbf24"
                  fontSize="13"
                >
                  {wire.sign}
                </text>
              )}
            </g>
          ))}
          {layout.boxes.map((box) => (
            <g key={box.key}>
              <NodeGlyph box={box} />
              <NodeLabel box={box} />
            </g>
          ))}
        </svg>
      </div>
    </section>
  )
}
