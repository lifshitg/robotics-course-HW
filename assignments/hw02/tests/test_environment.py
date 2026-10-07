"""Public tests of provided infrastructure. No student planner implementation."""
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'environment/src/hw02_sim'))
from hw02_sim.scenario import LAYOUTS,PILLAR_RADIUS,ROBOT_RADIUS,raw_map,validate_target,steering
from hw02_sim.world import world


@pytest.mark.parametrize('layout',list(LAYOUTS))
def test_world_and_raw_map_share_geometry(layout):
    xml=ET.fromstring(world(layout))
    pillars=[model for model in xml.findall('./world/model') if model.attrib['name'].startswith('pillar_')]
    assert len(pillars)==len(LAYOUTS[layout])
    grid=raw_map(layout)
    assert grid.shape==(240,320) and set(np.unique(grid))=={0,100}
    assert np.all(grid[[0,-1],:]==100) and np.all(grid[:,[0,-1]]==100)
    for model,(x,y) in zip(pillars,LAYOUTS[layout]):
        assert list(map(float,model.findtext('pose').split()))[:2]==[x,y]
        assert float(model.findtext('./link/collision/geometry/cylinder/radius'))==PILLAR_RADIUS
        assert grid[int(y/.025),int(x/.025)]==100
    robot=xml.find('./world/model[@name="hw02_robot"]')
    assert float(robot.findtext('./link[@name="base_link"]/collision/geometry/cylinder/radius'))==ROBOT_RADIUS
    for link in robot.findall('./link'):
        # Every physical contact geometry has a contact sensor.
        collision=link.find('collision')
        assert link.findtext('./sensor/contact/collision')==collision.attrib['name']
        xyz=list(map(float,link.findtext('pose').split()))[:3]
        if 'wheel' in link.attrib['name']:
            assert math.hypot(.08,abs(xyz[1])+.04/2)<ROBOT_RADIUS
        elif 'caster' in link.attrib['name']:
            assert abs(xyz[0])+.025<ROBOT_RADIUS
    assert grid[120,40]==0 and grid[120,280]==0
    # The published map is raw: this cell is outside the pillar but within robot radius.
    x,y=LAYOUTS[layout][0]
    assert grid[int(y/.025),int((x+.50)/.025)]==0


@pytest.mark.parametrize('frame,x,y,z',[('base_link',1,3,0),('map',float('nan'),3,0),('map',1,float('inf'),0),('map',0,3,0),('map',1,3,1)])
def test_bad_targets(frame,x,y,z):
    with pytest.raises(ValueError):validate_target(frame,x,y,z)


def test_steering_turns_before_driving_and_stops():
    assert steering((1,3,0),(1,3))==(0,0,0)
    linear,angular,_=steering((1,3,0),(1,4))
    assert linear==0 and 0<angular<=.8
    assert steering((1,3,0),(7,3))[0]==.2
    assert steering((1,3,0),(0,3))[0]==0
