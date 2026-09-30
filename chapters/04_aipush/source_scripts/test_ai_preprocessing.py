"""Synthetic tests only: no E-drive telemetry is loaded."""
import unittest
from unittest.mock import patch
from pathlib import Path
from tempfile import TemporaryDirectory
import sys
import numpy as np
import pandas as pd
sys.path.insert(0,str(Path(__file__).resolve().parent))
import preprocess_ai_driving_characteristics as m

def circle(start=.31,period=20.,hz=100):
    t=np.arange(0,period+1.01,1/hz);angle=2*np.pi*(start+t/period)
    d=pd.DataFrame({'t_sec':t,'normalizedCarPosition':np.mod(start+t/period,1),
        'x':50*np.cos(angle),'y':np.zeros(len(t)),'z':50*np.sin(angle),
        'heading':(angle+np.pi)%(2*np.pi)-np.pi,'pitch':0.,'roll':0.,
        'speedKmh':50*2*np.pi/period*3.6,'gas':.6,'brake':0.,'steerAngle':.03})
    for i,w in enumerate(m.WHEELS):
        d['SlipAngle_'+w]=i+1.;d['wheelSlip_'+w]=.1*(i+1);d['NdSlip_'+w]=.1*(i+1)
    return d

INFO={'config':'track+car','track':'track','vehicle':'car','setting':80,'run':'1','source':'synthetic.xlsx'}

class Tests(unittest.TestCase):
    def setUp(self):self.a=m.arguments([])
    def extracted(self,d=None):
        with patch.object(m.pd,'read_excel',return_value=circle() if d is None else d):
            return m.read_and_extract('synthetic.xlsx',self.a)
    def test_midlap_complete_circuit(self):
        lap,qc,_=self.extracted()
        self.assertAlmostEqual(qc['lap_time_s'],20,places=9)
        self.assertAlmostEqual(lap['_progress'].iloc[-1],1)
        self.assertEqual(qc['progress_regression_rows'],0)
    def test_exact_endpoint_interpolation(self):
        lap,qc,_=self.extracted(circle(period=20.003))
        self.assertAlmostEqual(qc['lap_time_s'],20.003,places=8)
        self.assertGreater(qc['endpoint_interpolation_correction_s'],0)
    def test_incomplete_and_time_reset_rejected(self):
        with self.assertRaisesRegex(ValueError,'No complete circuit'):self.extracted(circle().iloc[:100])
        d=circle();d.loc[100,'t_sec']=0
        with self.assertRaisesRegex(ValueError,'Timestamp regression'):self.extracted(d)
    def test_missing_optional_and_duplicate_time(self):
        d=circle()[['t_sec','normalizedCarPosition']];d=pd.concat([d.iloc[:3],d.iloc[2:]],ignore_index=True)
        lap,qc,_=self.extracted(d)
        self.assertEqual(qc['duplicate_timestamp_rows'],1)
        self.assertAlmostEqual(qc['lap_time_s'],20)
    def test_time_weighting(self):
        s=m.summarize(np.array([0.,1.,3.]),np.array([0.,2.,4.]),10)
        self.assertAlmostEqual(s['time_integral'],7.)
        self.assertAlmostEqual(s['time_mean'],7/3)
    def test_circular_mean(self):
        s=m.summarize(np.array([0.,1.,2.]),np.deg2rad([179,-179,179]),10,2*np.pi)
        self.assertLess(abs(abs(s['circular_mean'])-np.pi),.02)
        self.assertLess(s['circular_sd'],.04)
    def test_gap_not_bridged(self):
        z=m.interpolated(np.array([0.,.01,1.,1.01]),np.ones(4),np.array([.005,.5,1.005]),.1)
        self.assertTrue(np.isnan(z[1]));self.assertEqual(z[0],1)
        y=m.derivative(np.array([0.,1.,2.,np.nan,9.,10.,11.]),np.arange(7.))
        self.assertTrue(np.isnan(y[3]));np.testing.assert_allclose(y[[0,1,2,4,5,6]],1)
    def test_kinematic_circle(self):
        lap,_,_=self.extracted();grid,signals,_,_,_=m.derived_series(lap,self.a)
        mid=(grid>1)&(grid<19)
        np.testing.assert_allclose(signals['path_curvature_abs_m_inv'][mid],1/50,rtol=.01)
        np.testing.assert_allclose(signals['yaw_rate_rad_s'][mid],2*np.pi/20,rtol=.001)
        self.assertLess(np.nanmax(np.abs(signals['speed_derivative_m_s2'])),1e-9)
        wheel=signals['SlipAngle_rear_minus_front'];self.assertGreater(np.isfinite(wheel).mean(),.95)
        np.testing.assert_allclose(wheel[np.isfinite(wheel)],2)
    def test_absolute_profile_alignment(self):
        results=[]
        for start in [.13,.67]:
            lap,_,_=self.extracted(circle(start=start))
            grid,signals,_,_,_=m.derived_series(lap,self.a)
            results.append(m.spatial_profile(lap,grid,signals,self.a))
        # The complete circuit includes the same geographic positions despite different starts.
        np.testing.assert_allclose(results[0]['x'],results[1]['x'],atol=.05,equal_nan=True)
    def test_events(self):
        stats,events=m.event_rows(np.arange(6.),np.array([0,1,1,0,np.nan,1.]),.4,'gt','brake')
        self.assertEqual(stats['episode_count'],1);self.assertEqual(events[0]['duration_s'],3)
        self.assertTrue(events[0]['right_censored'])
    def test_spectrum(self):
        t=np.arange(0,10,.02);s=m.spectrum(np.sin(2*np.pi*2*t),50)
        self.assertAlmostEqual(s['dominant_frequency_hz'],2,places=6)
    def test_process_and_aggregate_overwrite(self):
        with patch.object(m.pd,'read_excel',return_value=circle()):
            result=m.process(INFO,self.a)
        features,qc,inventory,catalog,events,corr,profile,lap,ts=result
        self.assertGreater(len(features),1000)
        self.assertEqual(len(profile),1000)
        with TemporaryDirectory() as tmp:
            out=Path(tmp);m.aggregate(features,[profile],[INFO],out,self.a)
            m.aggregate(features,[profile],[INFO],out,self.a)
            s=pd.read_csv(out/'condition_summary.csv')
            self.assertTrue((s.n<=1).all());self.assertTrue(s.sample_sd.isna().all())
            m.write_csv([{'value':1}],out/'test.csv.gz');m.write_csv([{'value':2}],out/'test.csv.gz')
            self.assertEqual(pd.read_csv(out/'test.csv.gz').value.iloc[0],2)

if __name__=='__main__':unittest.main()
