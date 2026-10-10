export interface RunBannerProps {
  stopReason: string | null
  error: string | null
}

const BOX =
  'rounded border border-amber-800 bg-amber-950/40 px-3 py-2 text-sm text-amber-200'

/** What stopped the run, and what the API rejected. */
export function RunBanner({ stopReason, error }: RunBannerProps) {
  if (stopReason === null && error === null) return null

  return (
    <div role="alert" className={BOX}>
      {error !== null && <span className="block">{error}</span>}
      {stopReason !== null && <span className="block">Stopped: {stopReason}</span>}
    </div>
  )
}
