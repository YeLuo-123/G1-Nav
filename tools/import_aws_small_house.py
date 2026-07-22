#!/usr/bin/env python3
"""Convert the AWS RoboMaker Small House occupancy map into MuJoCo geometry."""

from pathlib import Path
import json
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.deps' / 'python'))

from PIL import Image

SOURCE = ROOT / '.deps/aws-small-house/maps/turtlebot3_waffle_pi/map.pgm'
SOURCE_LICENSE = ROOT / '.deps/aws-small-house/LICENSE'
SCENE = ROOT / 'simulation/aws_small_house_scene.xml'
TEXTURE = ROOT / 'simulation/aws_small_house_map.png'
GEOMETRY = ROOT / 'simulation/aws_small_house_geometry.json'
LICENSE = ROOT / 'simulation/AWS_SMALL_HOUSE_LICENSE'

RESOLUTION = 0.05
ORIGIN_X = -12.5
ORIGIN_Y = -12.5


def occupied_rectangles(image):
    """Merge identical horizontal occupied runs across adjacent image rows."""
    width, height = image.size
    pixels = image.load()
    active = {}
    rectangles = []

    for row in range(height):
        runs = []
        col = 0
        while col < width:
            if pixels[col, row] >= 100:
                col += 1
                continue
            start = col
            while col + 1 < width and pixels[col + 1, row] < 100:
                col += 1
            runs.append((start, col))
            col += 1

        current = {}
        for run in runs:
            if run in active:
                current[run] = active.pop(run)
            else:
                current[run] = row
        for (x0, x1), y0 in active.items():
            rectangles.append((x0, x1, y0, row - 1))
        active = current

    for (x0, x1), y0 in active.items():
        rectangles.append((x0, x1, y0, height - 1))
    return rectangles


def world_rectangle(rectangle, height):
    x0, x1, y0, y1 = rectangle
    cx = ORIGIN_X + (x0 + x1 + 1) * RESOLUTION / 2
    cy = ORIGIN_Y + (height - (y0 + y1 + 1) / 2) * RESOLUTION
    hx = max(0.03, (x1 - x0 + 1) * RESOLUTION / 2)
    hy = max(0.03, (y1 - y0 + 1) * RESOLUTION / 2)
    return [cx, cy, hx, hy]


def main():
    image = Image.open(SOURCE).convert('L')
    image.save(TEXTURE)
    rectangles = [
        world_rectangle(rectangle, image.height)
        for rectangle in occupied_rectangles(image)
    ]
    GEOMETRY.write_text(json.dumps({
        'source': 'aws-robotics/aws-robomaker-small-house-world',
        'license': 'MIT-0',
        'resolution': RESOLUTION,
        'origin': [ORIGIN_X, ORIGIN_Y],
        'spawn': [-4.0, -3.0],
        'rectangles': rectangles,
    }, indent=2))
    shutil.copyfile(SOURCE_LICENSE, LICENSE)

    geoms = []
    for index, (cx, cy, hx, hy) in enumerate(rectangles):
        material = 'wall_a' if index % 3 else 'wall_b'
        geoms.append(
            f'    <geom name="house_{index}" type="box" '
            f'pos="{cx:.4f} {cy:.4f} 0.8" '
            f'size="{hx:.4f} {hy:.4f} 0.8" material="{material}"/>'
        )

    xml = f'''<mujoco model="AWS RoboMaker Small House with Unitree G1">
  <include file="../.deps/unitree_rl_gym/resources/robots/g1_description/g1_12dof.xml"/>
  <statistic center="0 0 1" extent="13"/>
  <visual>
    <headlight diffuse=".75 .75 .75" ambient=".28 .28 .28"/>
    <rgba haze=".12 .15 .18 1"/>
    <global azimuth="135" elevation="-35"/>
  </visual>
  <asset>
    <texture name="house_map" type="2d" file="aws_small_house_map.png"/>
    <material name="floor_map" texture="house_map" texuniform="false" reflectance=".08"/>
    <material name="wall_a" rgba=".78 .72 .64 1"/>
    <material name="wall_b" rgba=".58 .64 .70 1"/>
    <material name="dynamic" rgba=".85 .25 .12 1"/>
  </asset>
  <worldbody>
    <light pos="0 0 10" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="12.5 12.5 .05" material="floor_map"/>
{chr(10).join(geoms)}
    <body name="dynamic_box" mocap="true" pos="0 -2.2 .45">
      <geom name="dynamic_box_geom" type="box" size=".40 .32 .45" material="dynamic"/>
    </body>
    <body name="dynamic_pedestrian" mocap="true" pos="-3 .8 .75">
      <geom name="dynamic_pedestrian_geom" type="cylinder" size=".26 .75" material="dynamic"/>
    </body>
  </worldbody>
</mujoco>
'''
    SCENE.write_text(xml)
    print(f'Generated {SCENE} with {len(rectangles)} merged collision boxes.')


if __name__ == '__main__':
    main()
