"""Select completedLaps == 1 (old protocol), without sector-time reconstruction."""
from pathlib import Path
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent
INPUT_DIR = str(ROOT / '80-ai')
INPUT_FILENAME = '801-aioriginal.xlsx'
OUTPUT_DIR = str(ROOT / '80-ai' / 'full_track_results')
OUTPUT_FILENAME = '801_lap.xlsx'
LAP_INDEX = 1


def select_lap(df, lap_index=1):
    if 'completedLaps' not in df:
        raise KeyError('Missing completedLaps; cannot apply the original lap-selection protocol.')
    laps = pd.to_numeric(df['completedLaps'], errors='coerce').to_numpy(float)
    if not np.isfinite(laps).all() or not np.equal(laps, np.floor(laps)).all():
        raise ValueError('completedLaps contains missing/non-integer values.')
    if np.any(np.diff(laps) < 0):
        raise ValueError('Lap counter decreases/resets; split sessions first.')
    ids = np.flatnonzero(laps == lap_index)
    if not len(ids):
        raise ValueError(f'No rows with completedLaps == {lap_index}; present: {np.unique(laps)}')
    i0, i1 = int(ids[0]), int(ids[-1])
    if np.any(np.diff(ids) != 1):
        raise ValueError('Selected lap is not a contiguous block.')
    entered = i0 > 0 and laps[i0 - 1] == lap_index - 1
    exited = i1 + 1 < len(laps) and laps[i1 + 1] == lap_index + 1
    out = df.iloc[i0:i1 + 1].copy().reset_index(drop=True)
    if 't_sec' in out:
        t = pd.to_numeric(out['t_sec'], errors='coerce').to_numpy(float)
        if not np.isfinite(t).all() or np.any(np.diff(t) < 0):
            raise ValueError('Selected-lap t_sec is invalid/decreasing; do not silently reorder.')
    lap_time = np.nan
    if exited and 'iLastTime_s' in df:
        value = pd.to_numeric(pd.Series([df.iloc[i1 + 1]['iLastTime_s']]), errors='coerce').iloc[0]
        if np.isfinite(value) and value > 0:
            lap_time = float(value)
    complete = bool(entered and exited)
    out['LapTime_s'] = lap_time
    out['lap_complete'] = complete
    audit = dict(raw_rows=len(df), selected_rows=len(out), selected_completedLaps=int(lap_index),
                 source_first_row_0based=i0, source_last_row_0based=i1,
                 entry_transition_observed=bool(entered), exit_transition_observed=bool(exited),
                 lap_complete=complete, lap_time_s=lap_time)
    return out, audit


def process_file(input_path, output_path, lap_index=1):
    input_path, output_path = Path(input_path), Path(output_path)
    if input_path.resolve() == output_path.resolve():
        raise ValueError('Output must not overwrite raw input.')
    out, audit = select_lap(pd.read_excel(input_path), lap_index)
    audit['input_file'] = str(input_path.resolve())
    if not audit['lap_complete']:
        raise ValueError(f'Incomplete lap {lap_index}: entry={audit["entry_transition_observed"]}, '
                         f'exit={audit["exit_transition_observed"]}. No full-track output written.')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    out.to_excel(output_path, index=False)
    pd.DataFrame([audit]).to_csv(output_path.with_suffix('.audit.csv'), index=False, encoding='utf-8-sig')
    print(f'[LAP] {input_path.name}: {len(out)} rows -> {output_path}')
    return audit


def main():
    return process_file(Path(INPUT_DIR) / INPUT_FILENAME, Path(OUTPUT_DIR) / OUTPUT_FILENAME, LAP_INDEX)


if __name__ == '__main__':
    main()
