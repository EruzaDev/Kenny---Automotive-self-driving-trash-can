"""Geometric sensor surrogates, not vendor SDK or photorealistic camera emulation."""
from dataclasses import dataclass
import numpy as np
from .geometry import ray_boxes, inside_rects, wrap_angle


@dataclass
class Scan:
    ranges: np.ndarray
    valid: np.ndarray
    angles: np.ndarray
    hits: np.ndarray


def lidar(world, pose, robot, rng, noise, dropout):
    angles = np.linspace(-np.pi, np.pi, 72, endpoint=False)
    theta = angles + pose[2]
    directions = np.column_stack((np.cos(theta), np.sin(theta), np.zeros(72)))
    ranges, index = ray_boxes([*pose[:2], robot.lidar_height], directions,
                              world.all_boxes(), robot.lidar_range)
    valid = rng.random(72) >= dropout
    ranges = np.clip(ranges + rng.normal(0, noise, 72), .02, robot.lidar_range)
    ranges = np.where(valid, ranges, robot.lidar_range)
    return Scan(ranges, valid, angles, (index >= 0) & valid)


def depth(world, pose, robot, rng, noise, dropout):
    az = np.linspace(-robot.camera_hfov_deg/2, robot.camera_hfov_deg/2, 36) * np.pi/180
    el = np.linspace(robot.camera_pitch_deg-robot.camera_vfov_deg/2,
                     robot.camera_pitch_deg+robot.camera_vfov_deg/2, 9) * np.pi/180
    azimuth, elevation = np.meshgrid(az+pose[2], el, indexing="ij")
    directions = np.stack((np.cos(elevation)*np.cos(azimuth),
                           np.cos(elevation)*np.sin(azimuth), np.sin(elevation)), axis=-1)
    rays = directions.reshape(-1, 3)
    distance, index = ray_boxes([*pose[:2], robot.camera_height], rays,
                                world.all_boxes(), robot.camera_range)
    # The ground occludes rays before obstacles beyond the ground intersection.
    ground = np.divide(-robot.camera_height, rays[:, 2],
                       out=np.full(len(rays), np.inf), where=rays[:, 2] < -1e-8)
    ground_xy = pose[:2] + rays[:, :2] * np.minimum(ground, robot.camera_range)[:, None]
    ground = np.where(inside_rects(ground_xy, world.cliffs), np.inf, ground)
    z = robot.camera_height + rays[:, 2]*distance
    hit = ((index >= 0) & (distance <= ground) & (distance >= robot.camera_min_range) &
           (z > .015) & (z < robot.height+.05))
    horizontal = distance * np.linalg.norm(rays[:, :2], axis=1)
    ranges = np.min(np.where(hit, horizontal, robot.camera_range).reshape(36, -1), axis=1)
    # Columns with any too-close occluding return are invalid, not known clear.
    too_close = ((index >= 0) & (distance < robot.camera_min_range)).reshape(36, -1).any(axis=1)
    valid = (rng.random(36) >= dropout) & ~too_close
    ranges = np.clip(ranges+rng.normal(0, noise, 36), 0., robot.camera_range)
    ranges = np.where(valid, ranges, robot.camera_range)
    return Scan(ranges, valid, az, hit.reshape(36, -1).any(axis=1) & valid)


def floor_scan(world, pose, robot, rng, dropout):
    angles = np.linspace(-robot.camera_hfov_deg/2, robot.camera_hfov_deg/2, 12)*np.pi/180
    # Expected visible ground distances from a finite set of downward depth rays.
    elevations = np.deg2rad(np.linspace(robot.camera_pitch_deg-robot.camera_vfov_deg/2,
                                        min(-5., robot.camera_pitch_deg+robot.camera_vfov_deg/2), 7))
    distances = robot.camera_height / np.maximum(-np.tan(elevations), .01)
    distances = distances[(distances >= robot.camera_min_range) & (distances < robot.camera_range)]
    if not len(distances):
        return Scan(np.full(12, robot.camera_range), np.zeros(12, bool), angles, np.zeros(12, bool))
    a, d = np.meshgrid(angles+pose[2], distances, indexing="ij")
    points = pose[:2] + np.stack((np.cos(a)*d, np.sin(a)*d), axis=-1)
    vectors = np.concatenate((points.reshape(-1, 2)-pose[:2],
                              np.full((points.size//2, 1), -robot.camera_height)), axis=1)
    lengths = np.linalg.norm(vectors, axis=1)
    measured, _ = ray_boxes([*pose[:2], robot.camera_height], vectors/lengths[:, None],
                            world.all_boxes(), robot.camera_range*2)
    visible = (measured >= lengths-.02).reshape(a.shape)
    drop = inside_rects(points, world.cliffs) & visible
    ranges = np.min(np.where(drop, d, robot.camera_range), axis=1)
    valid = visible.any(axis=1) & (rng.random(12) >= dropout)
    ranges = np.where(valid, ranges, robot.camera_range)
    return Scan(ranges, valid, angles, drop.any(axis=1) & valid)


def downward(world, pose, robot, rng, dropout):
    # Three added sensors mounted ahead of the conservative footprint.
    angles = np.deg2rad([-45., 0., 45.]) + pose[2]
    points = pose[:2] + (robot.radius+robot.cliff_lookahead)*np.column_stack((np.cos(angles), np.sin(angles)))
    hazard = inside_rects(points, world.cliffs)
    valid = rng.random(3) >= dropout
    return hazard, valid


def marker_visible(world, pose, robot, rng, dropout):
    delta = world.markers-pose[:2]
    ranges = np.linalg.norm(delta, axis=1)
    bearing = wrap_angle(np.arctan2(delta[:, 1], delta[:, 0])-pose[2])
    angle_down = np.rad2deg(np.arctan2(-robot.camera_height, np.maximum(ranges, .001)))
    candidates = np.flatnonzero((ranges < robot.marker_range) & (ranges > .2) &
                                (np.abs(bearing) < np.deg2rad(robot.camera_hfov_deg/2)) &
                                (angle_down > robot.camera_pitch_deg-robot.camera_vfov_deg/2) &
                                (angle_down < robot.camera_pitch_deg+robot.camera_vfov_deg/2))
    if not len(candidates) or rng.random() < dropout:
        return False
    vectors = np.column_stack((delta[candidates], np.full(len(candidates), -robot.camera_height)))
    lengths = np.linalg.norm(vectors, axis=1)
    distance, _ = ray_boxes([*pose[:2], robot.camera_height], vectors/lengths[:, None],
                            world.all_boxes(), robot.marker_range+robot.camera_height)
    return bool(np.any(distance >= lengths-.01))
