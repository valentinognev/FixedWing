export type Theme = 'light' | 'dark'

export const THEME_KEY = 'flightbench-theme'

/** Dark unless a previous visit stored light. */
export function readTheme(): Theme {
  try {
    return localStorage.getItem(THEME_KEY) === 'light' ? 'light' : 'dark'
  } catch {
    return 'dark'
  }
}

/** Plotly does not read CSS variables, so charts take these literals. */
export function chartChrome(theme: Theme): {
  tick: string
  grid: string
  line: string
  title: string
  paper: string
  font: string
} {
  if (theme === 'light') {
    return {
      tick: '#525252',
      grid: '#e5e5e5',
      line: '#d4d4d4',
      title: '#262626',
      paper: '#fafafa',
      font: '#404040',
    }
  }
  return {
    tick: '#a3a3a3',
    grid: '#262626',
    line: '#404040',
    title: '#e5e5e5',
    paper: '#0a0a0a',
    font: '#d4d4d4',
  }
}

/** Paint the document and keep the choice for the next visit. */
export function applyTheme(theme: Theme): void {
  document.documentElement.dataset.theme = theme
  try {
    localStorage.setItem(THEME_KEY, theme)
  } catch {
    // A private window can refuse storage; the page still switches.
  }
}
