/** `sliderGain / runGain` when that ratio is defined; otherwise `1`. */
export function nyquistScale(
  proportional: boolean,
  runGain: number,
  sliderGain: number,
): number {
  if (proportional && Number.isFinite(runGain) && runGain !== 0) {
    return sliderGain / runGain
  }
  return 1
}
