# flightbench

Web UI for the Flightbench control-law calibration bench. It never computes
flight dynamics: every number it renders comes from the FastAPI app on
`127.0.0.1:8765` (`flightbench.api:app`).

## Run

```bash
npm run dev     # Vite on :5173, /api proxied to http://127.0.0.1:8765
npm run build   # dist/, served at / by run_flightbench.sh
npm test        # vitest run (jsdom, Plotly is mocked by tests)
```

## Layout

| Path | Role |
|------|------|
| `src/types.ts` | The API's JSON shapes, mirrored verbatim (see the design spec's "API" section). |
| `src/api.ts` | `fetchPlanes` / `fetchTasks` / `fetchGains` / `postRun`; non-2xx becomes an `ApiError` with the server's `detail`. |
| `src/components/Plot.tsx` | The only place Plotly may be imported, so tests can mock it. |
| `src/__tests__/` | Vitest suites. |
