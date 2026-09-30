import unittest
import numpy as np
import pandas as pd
from compare_ai_assisted_full_track import compare


class ComparisonTests(unittest.TestCase):
    def test_unpaired_run_means(self):
        df = pd.DataFrame({'AIpush': [80]*5,
                           'condition': ['AI-only']*2+['optimizer-assisted']*3,
                           'driving_speed_mean': [10, 20, 20, 30, 40],
                           'driving_rows': [10000, 1, 1, 1, 1],
                           'tracking_heading_err_abs_mean_rad': [1]*5})
        groups, delta = compare(df)
        d = delta[delta.AIpush.eq(80)].iloc[0]
        self.assertEqual(d.ai_mean, 15)
        self.assertEqual(d.assisted_mean, 30)
        self.assertEqual(d.delta_assisted_minus_ai, 15)
        self.assertEqual(d.relative_change_pct, 100)
        self.assertEqual(d.ai_n, 2)
        self.assertEqual(d.assisted_n, 3)
        self.assertEqual(set(groups.metric), {'driving_speed_mean'})
        self.assertAlmostEqual(d.ai_std, np.std([10, 20], ddof=1))

    def test_missing_invalid_zero_and_percentage_points(self):
        df = pd.DataFrame({'AIpush': [80, 80, 80],
                           'condition': ['AI-only', 'optimizer-assisted', 'optimizer-assisted'],
                           'driving_pct_a_lt_m3': [0, 4, np.nan]})
        groups, delta = compare(df)
        d = delta[delta.AIpush.eq(80)].iloc[0]
        self.assertEqual(d.delta_assisted_minus_ai, 4)
        self.assertTrue(pd.isna(d.relative_change_pct))
        self.assertTrue(pd.isna(d.assisted_std))
        self.assertEqual(d.delta_unit, 'percentage points')
        self.assertEqual(d.assisted_n, 1)
        absent = delta[delta.AIpush.eq(85)].iloc[0]
        self.assertTrue(pd.isna(absent.delta_assisted_minus_ai))
        self.assertEqual(absent.comparison_status, 'missing_group_or_metric')


if __name__ == '__main__':
    unittest.main()
