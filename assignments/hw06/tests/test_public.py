"""Public data/geometry checks only; never import a student or reference algorithm."""
from pathlib import Path
import hashlib
import numpy as np
import pytest
from hw06_support import load_dataset, load_development_truth, Grid, endpoints, ray_cells, scan_cells, distance_field
from hw06_support.geometry import integrate_odometry, wrap_angle
from hw06_support.evaluation import trajectory_metrics

@pytest.mark.parametrize('name',['development','assessment'])
def test_dataset_schema_and_checksum(name):
    d=load_dataset(name)
    assert len(d)==500 and len(d.angles)==181
    np.testing.assert_allclose(np.diff(d.timestamps),.5)
    assert d.timestamps[-1]==249.5
    assert np.all(d.odometry[0]==0)
    from hw06_support.data import ROOT
    path=ROOT/'assignments/hw06/data'/name/'sensors.npz'
    assert hashlib.sha256(path.read_bytes()).hexdigest()==d.metadata['sensors_sha256']
    assert .005 < np.mean(d.status==0) < .015
    assert np.any(d.status==2)
    if name=='assessment':
        assert {p.name for p in path.parent.iterdir()}=={'metadata.json','sensors.npz'}


def test_development_answers_and_initial_frame():
    d=load_dataset('development'); truth=load_development_truth()
    assert truth['trajectory'].shape==(500,3) and truth['occupancy'].shape==(100,120)
    assert set(np.unique(truth['occupancy']))=={0,100}
    np.testing.assert_array_equal(truth['timestamps'],d.timestamps)
    np.testing.assert_array_equal(truth['trajectory'][0],[0,0,0])
    poses=integrate_odometry(d)
    assert np.isfinite(poses).all() and poses.shape==(500,3)
    assert trajectory_metrics(poses,truth['trajectory'])['position_rmse_m']>0


def test_grid_conversion_and_outside_mask():
    g=Grid(); row,col,valid=g.indices([[-5.95,-4.95],[5.95,4.95],[6.,0],[-6.01,0],[np.nan,0]])
    np.testing.assert_array_equal(valid,[True,True,False,False,False])
    np.testing.assert_array_equal(row[:2],[0,99]); np.testing.assert_array_equal(col[:2],[0,119])
    np.testing.assert_allclose(g.centers(row[:2],col[:2]),[[-5.95,-4.95],[5.95,4.95]])


def test_projection_and_angle_wrapping():
    origin,ends=endpoints([1,2,np.pi/2],[1],[0],[.2,0,np.pi/2])
    np.testing.assert_allclose(origin,[1,2.2]); np.testing.assert_allclose(ends,[[0,2.2]],atol=1e-12)
    assert np.isclose(wrap_angle(3*np.pi),-np.pi)


def test_ray_axes_diagonal_and_clipping():
    g=Grid(origin=(0.,0.),resolution=1.,width=4,height=4)
    np.testing.assert_array_equal(ray_cells([.5,.5],[3.5,.5],g),[[0,0],[0,1],[0,2],[0,3]])
    np.testing.assert_array_equal(ray_cells([.5,.5],[.5,3.5],g),[[0,0],[1,0],[2,0],[3,0]])
    np.testing.assert_array_equal(ray_cells([.5,.5],[3.5,3.5],g),[[0,0],[1,1],[2,2],[3,3]])
    assert len(ray_cells([.5,.5],[.5,.5],g))==0  # no positive-length segment
    clipped=ray_cells([.5,.5],[10,.5],g)
    np.testing.assert_array_equal(clipped,[[0,0],[0,1],[0,2],[0,3]])
    assert scan_cells([0,0],np.empty((0,2)),g)[0].size==0
    with pytest.raises(ValueError): ray_cells([np.nan,0],[1,1],g)


def test_distance_units_and_empty_mask():
    mask=np.zeros((3,3),dtype=bool)
    assert np.isinf(distance_field(mask,.1)).all()
    mask[1,1]=True; field=distance_field(mask,.1)
    assert field[1,1]==0 and np.isclose(field[1,2],.1) and np.isclose(field[0,0],np.sqrt(2)*.1)


def test_evaluation_no_alignment_and_invalid_results():
    truth=np.zeros((3,3)); estimate=truth.copy(); estimate[:,0]=2.; estimate[:,2]=2*np.pi
    result=trajectory_metrics(estimate,truth)
    assert result['position_rmse_m']==2 and result['heading_rmse_rad']==0
    with pytest.raises(ValueError): trajectory_metrics(np.full((3,3),np.nan),truth)
    with pytest.raises(ValueError): trajectory_metrics(np.zeros((2,3)),truth)


def test_replay_rejects_future_histories_and_assessment_overlays():
    from hw06_support.display import validate_replay, LiveDisplay
    meta=load_dataset('assessment').metadata
    arrays={'timestamps':np.array([0.,.5]),'ess':np.array([2.,2.]),'resampled':np.array([False,False]),
        'snapshot_indices':np.array([0,1]),'snapshot_maps':np.zeros((2,100,120)),
        'snapshot_trajectories':np.zeros((2,2,3)),'snapshot_particles':np.zeros((2,2,3))}
    arrays['snapshot_trajectories'][0,1]=np.nan
    validate_replay(arrays,meta)
    arrays['snapshot_trajectories'][0,1]=[1,2,3]
    with pytest.raises(ValueError,match='future'): validate_replay(arrays,meta)
    with pytest.raises(ValueError,match='development-only'): LiveDisplay(meta,load_development_truth())
