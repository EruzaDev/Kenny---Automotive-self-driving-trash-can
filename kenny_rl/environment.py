"""Versioned editor maps. Occupancy data is ROS row-major, bottom row first."""
import json
import math
from pathlib import Path
import numpy as np
from .world import World
from .geometry import circle_boxes, circle_rects


def local_point(point, origin):
    x, y = np.asarray(point[:2], dtype=float) - origin[:2]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return np.array([c*x+s*y, -s*x+c*y])


def destination_goal(env):
    """Resolve the authoritative marker selection, preserving legacy coordinate goals."""
    mid = env.get('destination_marker_id')
    if mid is None:
        return list(env['goal'])
    if type(mid) is not int:
        raise ValueError('Destination marker ID must be an integer')
    marker = next((m for m in env.get('markers', []) if m['id'] == mid), None)
    if marker is None:
        raise ValueError('Destination ArUco marker is missing')
    offset = env.get('destination_offset_m', .5 if marker['mount'] == 'wall' else 0.)
    if not isinstance(offset, (int, float)) or not math.isfinite(offset) or not 0 <= offset <= 2:
        raise ValueError('Destination approach offset must be 0–2 meters')
    return [marker['x']+math.cos(marker['yaw'])*offset, marker['y']+math.sin(marker['yaw'])*offset]


def validate_environment(env):
    if env.get('version') != 1 or env.get('frame_id') != 'map' or env.get('units') != 'meters':
        raise ValueError('Expected version 1, frame_id map and units meters')
    if env.get('dictionary') != 'DICT_4X4_50':
        raise ValueError('Supported dictionary: DICT_4X4_50')
    grid = env['grid']
    w, h, r = grid['width'], grid['height'], grid['resolution']
    if type(w) is not int or type(h) is not int or not (1 <= w <= 4000 and 1 <= h <= 4000) or w*h > 4_000_000:
        raise ValueError('Grid dimensions invalid or too large (maximum 4 million cells)')
    if not isinstance(r, (int, float)) or not math.isfinite(r) or not .005 <= r <= 1:
        raise ValueError('Resolution must be 0.005–1 meters')
    if len(grid['origin']) != 3 or not np.isfinite(grid['origin']).all():
        raise ValueError('Origin must contain finite x, y, yaw')
    data = np.asarray(grid['data'])
    if data.size != w*h or data.ndim != 1 or not np.isin(data, [-1, 0, 100]).all():
        raise ValueError('Occupancy must contain width*height values of -1, 0 or 100')
    ids = set()
    for marker in env.get('markers', []):
        mid = marker['id']
        if type(mid) is not int or not 0 <= mid < 50 or mid in ids:
            raise ValueError('DICT_4X4_50 marker IDs must be unique integers from 0 to 49')
        ids.add(mid)
        if env.get('dictionary') != 'DICT_4X4_50':
            raise ValueError('Supported dictionary: DICT_4X4_50')
        values = [marker[k] for k in ('x', 'y', 'z', 'yaw', 'size')]
        if not np.isfinite(values).all() or not .01 <= marker['size'] <= 1 or marker['z'] < 0 or marker['mount'] not in ('floor', 'wall'):
            raise ValueError('Invalid marker pose, mounting or size')
        p = local_point(values[:2], grid['origin'])
        if np.any(p < 0) or p[0] > w*r or p[1] > h*r:
            raise ValueError('Marker lies outside map')
    for obj in env.get('obstacles', []):
        if obj['kind'] not in ('wall', 'obstacle', 'prohibited'):
            raise ValueError('Unknown obstacle kind')
        if not np.isfinite([obj[k] for k in ('x','y','width','depth','bottom','height')]).all() or min(obj['width'], obj['depth'], obj['height']) <= 0 or obj['bottom'] < 0:
            raise ValueError('Obstacle dimensions must be positive and finite')
        corners = [local_point([obj['x']+dx,obj['y']+dy],grid['origin']) for dx in (0,obj['width']) for dy in (0,obj['depth'])]
        if any(p[0] < -1e-7 or p[1] < -1e-7 or p[0] > w*r+1e-7 or p[1] > h*r+1e-7 for p in corners):
            raise ValueError('Obstacle lies outside map')
    for key in ('start', 'goal'):
        if len(env[key]) != 2 or not np.isfinite(env[key]).all():
            raise ValueError(f'{key} must contain finite x/y')
    destination_goal(env)
    return env


def rasterize(env):
    validate_environment(env)
    g = env['grid']; r = g['resolution']
    data = np.asarray(g['data'], dtype=np.int16).reshape(g['height'], g['width']).copy()
    lx, ly = np.meshgrid((np.arange(g['width'])+.5)*r, (np.arange(g['height'])+.5)*r)
    c,s = math.cos(g['origin'][2]), math.sin(g['origin'][2])
    x = g['origin'][0]+c*lx-s*ly; y = g['origin'][1]+s*lx+c*ly
    for o in env.get('obstacles', []):
        mask = (x >= o['x']-r/2) & (x <= o['x']+o['width']+r/2) & (y >= o['y']-r/2) & (y <= o['y']+o['depth']+r/2)
        data[mask] = 100
    return data


def merged_rectangles(mask, resolution):
    """Merge identical horizontal runs across neighboring rows."""
    active = {}; result = []
    for y in range(mask.shape[0]+1):
        runs = []
        if y < mask.shape[0]:
            edges = np.diff(np.r_[False, mask[y], False].astype(int))
            runs = list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))
        current = set(runs)
        for run in list(active):
            if run not in current:
                start = active.pop(run)
                result.append([run[0]*resolution,start*resolution,run[1]*resolution,y*resolution])
        for run in runs:
            active.setdefault(run,y)
    return result


def load_environment(source, robot):
    env = json.loads(Path(source).read_text()) if isinstance(source,(str,Path)) else source
    validate_environment(env)
    g=env['grid']; r=g['resolution']; width=g['width']*r; height=g['height']*r
    size=max(width,height); boxes=[]; kinds=[]; cliffs=[]
    base=np.asarray(g['data']).reshape(g['height'],g['width'])
    for x0,y0,x1,y1 in merged_rectangles(base != 0,r):
        boxes.append([x0,y0,0,x1,y1,3]); kinds.append('wall')
    for b in ([0,0,0,width,.02,3],[0,0,0,.02,height,3],
              [width-.02,0,0,width,height,3],[0,height-.02,0,width,height,3]):
        boxes.append(b); kinds.append('wall')
    for o in env.get('obstacles',[]):
        pts=np.array([local_point([o['x']+dx,o['y']+dy],g['origin']) for dx in (0,o['width']) for dy in (0,o['depth'])])
        lo=pts.min(axis=0); hi=pts.max(axis=0)
        if o['kind']=='prohibited': cliffs.append([*lo,*hi])
        else:
            boxes.append([*lo,o['bottom'],*hi,o['bottom']+o['height']])
            kinds.append('wall' if o['kind']=='wall' else ('overhang' if o['bottom'] else 'bag'))
    boxes=np.array(boxes,dtype=float).reshape(-1,6); cliffs=np.array(cliffs,dtype=float).reshape(-1,4)
    start=local_point(env['start'],g['origin']); goal=local_point(destination_goal(env),g['origin'])
    for name,p in [('start',start),('goal',goal)]:
        if np.any(p <= robot.radius) or p[0]>=width-robot.radius or p[1]>=height-robot.radius or circle_boxes(p,robot.radius+.05,boxes,robot.height) or circle_rects(p,robot.radius+.05,cliffs):
            raise ValueError(f'{name} must be inside known free space with robot clearance')
    markers=np.array([local_point([m['x'],m['y']],g['origin']) for m in env.get('markers',[])],dtype=float).reshape(-1,2)
    world=World(size,boxes,kinds,cliffs,markers,np.empty((0,5)),start,goal,width,height)
    world.map_origin=np.array(g['origin']); world.marker_metadata=env.get('markers',[])
    world.destination_marker_id=env.get('destination_marker_id')
    marker=next((m for m in env.get('markers',[]) if m['id']==world.destination_marker_id),None)
    world.destination_position=local_point([marker['x'],marker['y']],g['origin']) if marker else goal.copy()
    return world
