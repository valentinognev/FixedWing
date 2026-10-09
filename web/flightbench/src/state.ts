import type { Law, PlaneInfo } from './types'

/** What the user has picked: one plane, law, aero model, task and trim point. */
export interface Selection {
  plane: string
  law: Law
  aero: string
  task: string
  trim: { vt_mps: number; altitude_m: number }
}

export type SelectionAction =
  | { type: 'planeChanged'; plane: PlaneInfo }
  | { type: 'lawChanged'; law: Law }
  | { type: 'aeroChanged'; aero: string }
  | { type: 'taskChanged'; task: string }
  | { type: 'trimChanged'; trim: Selection['trim'] }

export function planeChanged(plane: PlaneInfo): SelectionAction {
  return { type: 'planeChanged', plane }
}

export function lawChanged(law: Law): SelectionAction {
  return { type: 'lawChanged', law }
}

export function aeroChanged(aero: string): SelectionAction {
  return { type: 'aeroChanged', aero }
}

export function taskChanged(task: string): SelectionAction {
  return { type: 'taskChanged', task }
}

export function trimChanged(trim: Selection['trim']): SelectionAction {
  return { type: 'trimChanged', trim }
}

export function selectionReducer(
  state: Selection,
  action: SelectionAction,
): Selection {
  switch (action.type) {
    case 'planeChanged': {
      const { plane } = action
      return {
        ...state,
        plane: plane.id,
        law: plane.laws.includes(state.law) ? state.law : plane.laws[0],
        aero: plane.aero_models[0],
        trim: { ...plane.default_trim },
      }
    }
    case 'lawChanged':
      return { ...state, law: action.law }
    case 'aeroChanged':
      return { ...state, aero: action.aero }
    case 'taskChanged':
      return { ...state, task: action.task }
    case 'trimChanged':
      return { ...state, trim: { ...action.trim } }
    default:
      return state
  }
}
