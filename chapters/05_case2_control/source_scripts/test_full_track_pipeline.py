"""Small regression tests. No production telemetry is read."""
import tempfile
from pathlib import Path
import unittest
import numpy as np
import pandas as pd
import Data_pre_process as dp
import main4only_batch_analyze_driving_v2 as driving
import main4only_track_analyze_driving_v2 as tracking
import run_pipeline_from_original as pipeline


class FullTrackTests(unittest.TestCase):
    def frame(self):
        return pd.DataFrame(dict(completedLaps=[0, 1, 1, 1, 1, 1, 2],
                                 t_sec=np.arange(7) * 0.01, iLastTime_s=[0]*6+[1.23],
                                 normalizedCarPosition=[.9, 0, .2, .4, .6, .99, 0],
                                 speedKmh=[36]*7, x=[0]*7, y=[0]*7, z=[0]*7))

    def test_lap_no_sector_columns(self):
        lap, audit = dp.select_lap(self.frame())
        self.assertEqual(len(lap), 5)
        self.assertTrue(audit['lap_complete'])
        self.assertEqual(audit['lap_time_s'], 1.23)

    def test_incomplete_and_reset(self):
        _, audit = dp.select_lap(self.frame().iloc[:-1])
        self.assertFalse(audit['lap_complete'])
        bad = self.frame()
        bad.loc[3, 'completedLaps'] = 0
        with self.assertRaises(ValueError):
            dp.select_lap(bad)

    def test_global_only_and_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            raw, lap = root/'source.xlsx', root/'lap.xlsx'
            self.frame().to_excel(raw, index=False)
            for _ in range(2):
                dp.process_file(raw, lap)
            result = driving.analyze_one_file(str(lap))
            self.assertEqual(result.label.tolist(), ['GLOBAL'])
            self.assertEqual(result.speed_mean.iloc[0], 36)
            spline = root/'spline.csv'
            pd.DataFrame(dict(x=[0, 1, 1, 0], y=[0]*4, z=[0, 0, 1, 1])).to_csv(spline, index=False)
            tracking.DRIVE_XLSX = str(lap)
            tracking.SPLINE_CSV = str(spline)
            tracking.OUT_XLSX = str(root/'tracking.xlsx')
            for _ in range(2):
                tracking.main()
            result = pd.read_excel(tracking.OUT_XLSX)
            self.assertEqual(result.label.tolist(), ['GLOBAL'])
            self.assertEqual(result.cte_xz_mean_m.iloc[0], 0)
            self.assertTrue(pd.isna(result.prog_err_abs_mean.iloc[0]))
            self.assertNotIn('sector_index', driving.analyze_one_file(str(lap)).columns)

    def test_discovery_ignores_reports(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            folder = root/'80-ai'
            folder.mkdir()
            for name in ['801-aioriginal.xlsx', '801.xlsx', '801_main4_sector.xlsx', '~$801-aioriginal.xlsx']:
                (folder/name).touch()
            self.assertEqual([p.name for p in pipeline.discover(root)], ['801-aioriginal.xlsx'])


if __name__ == '__main__':
    unittest.main()
