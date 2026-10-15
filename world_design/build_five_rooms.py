#!/usr/bin/env python3
"""Generate a five-room Gazebo world, keeping the existing mapping topic/bounds."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET


PALETTE = {
    'floor': '0.77 0.78 0.74 1', 'wall': '0.88 0.89 0.84 1',
    'cap': '0.18 0.25 0.28 1', 'teal': '0.15 0.46 0.43 1',
    'blue': '0.22 0.37 0.51 1', 'orange': '0.76 0.36 0.19 1',
    'wood': '0.63 0.45 0.29 1', 'metal': '0.19 0.24 0.27 1',
    'white': '0.94 0.94 0.88 1', 'gold': '0.88 0.65 0.27 1',
    'screen': '0.10 0.23 0.29 1', 'leaf': '0.24 0.43 0.29 1',
    'terracotta': '0.65 0.36 0.26 1', 'paper': '0.88 0.85 0.74 1',
}


def sub(parent, tag, text=None, **attrs):
    result = ET.SubElement(parent, tag, attrs)
    if text is not None:
        result.text = str(text)
    return result


def material(visual, color):
    mat = sub(visual, 'material')
    rgba = PALETTE.get(color, color)
    sub(mat, 'ambient', rgba)
    sub(mat, 'diffuse', rgba)
    sub(mat, 'specular', '0.08 0.08 0.08 1')


def box(link, name, xyz, size, color, collision=True, yaw=0):
    for kind in (('collision', 'visual') if collision else ('visual',)):
        item = sub(link, kind, name=name + ('_collision' if kind == 'collision' else '_visual'))
        sub(item, 'pose', ' '.join(map(str, (*xyz, 0, 0, yaw))))
        sub(sub(sub(item, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))
        if kind == 'visual':
            material(item, color)
            if not collision:
                sub(item, 'cast_shadows', 'false')


def model(world, name):
    item = sub(world, 'model', name=name)
    sub(item, 'static', 'true')
    return sub(item, 'link', name='body')


def wall(link, name, x, y, sx, sy):
    box(link, name, (x, y, .75), (sx, sy, 1.5), 'wall')
    box(link, name + '_cap', (x, y, 1.515), (sx, sy, .03), 'cap', False)
    # Skirting is inside the wall footprint; it does not alter the LiDAR boundary.
    box(link, name + '_skirt', (x, y, .10), (sx + .002, sy + .002, .10), 'cap', False)


def horizontal_with_door(link, name, lo, hi, y, door_x, width=1.8):
    for suffix, a, b in [('left', lo, door_x - width / 2), ('right', door_x + width / 2, hi)]:
        wall(link, name + '_' + suffix, (a + b) / 2, y, b - a, .18)
    for i, x in enumerate((door_x - width / 2, door_x + width / 2)):
        box(link, name + f'_jamb_{i}', (x, y, .75), (.045, .21, 1.5), 'teal', False)


def cabinet(link, name, x, y, sx, sy, color, height=.85, yaw=0):
    box(link, name, (x, y, height / 2 + .05), (sx, sy, height), color, yaw=yaw)
    box(link, name + '_top', (x, y, height + .065), (sx, sy, .03), 'white', False, yaw=yaw)
    # Shallow door faces and handles follow the existing cabinet body.
    for i, dx in enumerate((-sx / 4, sx / 4)):
        for suffix, lx, ly, z, size, shade in [
            ('panel', dx, -sy / 2 - .008, height * .65,
             (sx / 2 - .035, .012, height * .48), color),
            ('handle', dx + sx * .12, -sy / 2 - .025, height * .72,
             (.025, .035, .13), 'gold')]:
            box(link, f'{name}_{suffix}_{i}',
                (x + lx * math.cos(yaw) - ly * math.sin(yaw),
                 y + lx * math.sin(yaw) + ly * math.cos(yaw), z), size, shade, False, yaw)


def table(link, name, x, y, sx, sy, color, height=.82):
    """Real tabletop and four legs, with no solid collision filling the underside."""
    box(link, name + '_tabletop', (x, y, height), (sx, sy, .08), color)
    leg_height = height - .04 - .05
    for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        box(link, name + f'_leg_{i}',
            (x + dx * (sx / 2 - .12), y + dy * (sy / 2 - .12), .05 + leg_height / 2),
            (.12, .12, leg_height), 'metal')


def chair(link, name, x, y, color, back_sign=1):
    """Seat/backrest above the scan plane; four legs intersect it."""
    box(link, name + '_seat', (x, y, .52), (.56, .56, .08), color)
    box(link, name + '_backrest', (x, y + back_sign * .25, .82), (.56, .08, .60), color)
    box(link, name + '_cushion', (x, y - back_sign * .015, .57), (.49, .46, .025), color, False)
    box(link, name + '_back_inset', (x, y + back_sign * .199, .86), (.43, .025, .35), 'cap', False)
    for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        box(link, name + f'_leg_{i}', (x + dx * .20, y + dy * .20, .265),
            (.09, .09, .43), 'metal')


def open_shelf(link, name, x, y, sx=.85, sy=2.0):
    """Open rack: posts and low boxes are visible to LiDAR, upper shelves are not."""
    for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        box(link, name + f'_post_{i}', (x + dx * (sx / 2 - .06), y + dy * (sy / 2 - .06), .95),
            (.10, .10, 1.8), 'metal')
    for i, height in enumerate((.20, .85, 1.50)):
        box(link, name + f'_shelf_{i}', (x, y, height), (sx, sy, .055), 'orange')
    for i, dy in enumerate((-.55, .55)):
        box(link, name + f'_low_box_{i}', (x, y + dy, .38), (.42, .46, .30), 'wood')
        box(link, name + f'_upper_box_{i}', (x, y + dy, 1.08), (.48, .50, .40), 'teal')
    box(link, name + '_top_box', (x, y, 1.68), (.45, .60, .30), 'gold')


def floor_number(link, number, x, y):
    segments = {
        'a': (0, .42, .44, .065), 'g': (0, 0, .44, .065), 'd': (0, -.42, .44, .065),
        'f': (-.22, .21, .065, .42), 'b': (.22, .21, .065, .42),
        'e': (-.22, -.21, .065, .42), 'c': (.22, -.21, .065, .42),
    }
    for segment in {1:'bc', 2:'abged', 3:'abgcd', 4:'fgbc', 5:'afgcd'}[number]:
        dx, dy, sx, sy = segments[segment]
        box(link, f'room_{number}_mark_{segment}', (x + dx, y + dy, .070), (sx, sy, .003), 'cap', False)


def monitor(link, name, x, y, desk_z, width=.58):
    box(link, name + '_base', (x, y, desk_z + .065), (.30, .22, .04), 'metal')
    box(link, name + '_stem', (x, y, desk_z + .22), (.045, .045, .28), 'metal')
    box(link, name + '_frame', (x, y, desk_z + .42), (width, .075, .37), 'metal')
    box(link, name + '_screen', (x, y - .040, desk_z + .42), (width - .045, .009, .325), 'screen', False)
    for i, length in enumerate((.32, .22, .28)):
        box(link, name + f'_ui_{i}', (x - .04, y - .046, desk_z + .50 - i * .07),
            (length, .004, .012), 'teal' if i else 'gold', False)
    box(link, name + '_keyboard', (x, y - .30, desk_z + .06), (.42, .15, .025), 'metal')


def plant(link, name, x, y):
    box(link, name + '_pot', (x, y, .29), (.40, .40, .48), 'terracotta')
    box(link, name + '_soil', (x, y, .535), (.34, .34, .014), 'wood', False)
    box(link, name + '_stem', (x, y, .85), (.045, .045, .65), 'wood')
    for i in range(6):
        angle = i * math.pi / 3
        box(link, name + f'_leaf_{i}', (x + .12 * math.cos(angle), y + .12 * math.sin(angle),
            .79 + .10 * (i % 3)), (.38, .12, .07), 'leaf', False, angle)


def floor_finishes(world):
    """Thin decorative surfaces; the existing ground collision stays at z=0.05."""
    link = model(world, 'interior_floor_details')
    # Wood boards in the meeting room and office, staggered without external textures.
    for room, bounds in [(3, (3.42, 9.88, 1.52, 7.88)), (4, (-9.88, -.12, -7.88, -1.52))]:
        xmin, xmax, ymin, ymax = bounds
        for row in range(math.ceil((ymax - ymin) / .38)):
            y = ymin + row * .38
            for col in range(math.ceil((xmax - xmin) / 1.65) + 1):
                a = max(xmin, xmin + col * 1.65 - (row % 2) * .825)
                b = min(xmax, xmin + (col + 1) * 1.65 - (row % 2) * .825)
                if b - a < .02:
                    continue
                shade = ((row * 7 + col * 3) % 5) * .014
                color = f'{.68 + shade} {.54 + shade} {.39 + shade} 1'
                height = min(.38, ymax - y)
                box(link, f'wood_{room}_{row}_{col}', ((a+b)/2, y+height/2, .057),
                    (b-a-.009, height-.009, .003), color, False)
    # Rugs are floor graphics, below the scan plane, with no raised edges.
    for name, x, y, sx, sy, color in [
        ('meeting_rug', 6.6, 5.5, 4.3, 3.6, '.37 .48 .53 1'),
        ('office_rug', -5.4, -5.3, 7.8, 3.5, '.53 .60 .56 1')]:
        box(link, name, (x, y, .061), (sx, sy, .002), color, False)
        for side in (-1, 1):
            box(link, name + f'_border_y_{side}', (x, y+side*(sy/2-.08), .064),
                (sx-.12, .018, .002), 'paper', False)
            box(link, name + f'_border_x_{side}', (x+side*(sx/2-.08), y, .064),
                (.018, sy-.12, .002), 'paper', False)
    for room, x, y, color in [(1,-6.65,2.5,'teal'), (2,0,2.5,'gold'),
                             (3,6.65,2.5,'blue'), (4,-5,-2.5,'teal'), (5,5,-2.5,'orange')]:
        box(link, f'entry_{room}', (x,y,.063), (1.3,1.3,.003), 'white', False)
        box(link, f'entry_accent_{room}', (x-.58,y,.067), (.07,1.15,.002), color, False)


def interior_details(world):
    """Furnish the five original rooms while keeping their entry aisles open."""
    architecture = model(world, 'interior_wall_details')
    # Inset wall panels and colour bands identify rooms without closing sight lines.
    for number, x, y, width, color in [(1,-6.65,7.87,5.7,'teal'), (2,0,7.87,5.7,'gold'),
                                      (3,6.65,7.87,5.7,'blue'), (4,-5,-7.87,8.9,'teal'),
                                      (5,5,-7.87,8.9,'orange')]:
        box(architecture, f'wall_panel_{number}', (x,y,.98), (width,.035,.81), 'white')
        box(architecture, f'wall_band_{number}', (x,y,.62), (width,.045,.13), color, False)
        for i in range(4):
            box(architecture, f'panel_seam_{number}_{i}', (x-width/2+(i+1)*width/5,y,.99),
                (.012,.048,.64), 'floor', False)

    lab = model(world, 'laboratory_equipment')
    table(lab, 'rear_bench', -6.7, 7.22, 3.3, .75, 'white', .86)
    monitor(lab, 'lab_computer', -7.55, 7.32, .86)
    box(lab, 'analyser', (-6.4,7.25,1.12), (.65,.44,.44), 'white')
    box(lab, 'analyser_screen', (-6.4,7.02,1.16), (.43,.015,.22), 'screen', False)
    for i in range(3):
        box(lab, f'sample_holder_{i}', (-5.70+i*.15,7.13,1.03), (.075,.12,.25), 'teal')
    box(lab, 'island_mat', (-6.05,5.1,.929), (.64,.60,.012), 'cap', False)
    for i in range(3):
        box(lab, f'island_sample_{i}', (-6.2+i*.16,5.1,1.025), (.085,.12,.18), 'gold')
    plant(lab, 'lab_plant', -9.2, 2.35)

    storage = model(world, 'storage_details')
    # Faces, shelf dividers and stock give the existing cabinet racks readable depth.
    for side, x in [('west',-2.2), ('east',2.2)]:
        for j, y in enumerate((4.95,5.7,6.45)):
            for k, z in enumerate((1.43,1.81)):
                box(storage, f'{side}_stock_{j}_{k}', (x,y,z), (.57,.59,.32),
                    'wood' if (j+k)%2 else 'teal')
                box(storage, f'{side}_label_{j}_{k}', (x,y-.301,z), (.22,.01,.10), 'paper', False)
        for k,z in enumerate((1.25,1.63,2.0)):
            box(storage, f'{side}_shelf_{k}', (x,5.7,z), (.79,2.53,.045), 'metal')
        for j,y in enumerate((4.48,6.92)):
            box(storage, f'{side}_upright_{j}', (x, y,1.58), (.79,.045,.91), 'orange')
    for i,x in enumerate((-.7,.65)):
        box(storage, f'crate_strap_{i}', (x,6.8,.724), (.09,.80,.012), 'metal', False)
    cabinet(storage, 'dispatch_counter', -2.35, 2.5, .65, 1.0, 'orange', .85)
    box(storage, 'dispatch_parcel', (-2.35,2.5,1.08), (.45,.5,.30), 'paper')

    meeting = model(world, 'meeting_details')
    cabinet(meeting, 'credenza', 6.6, 7.43, 2.6, .55, 'blue', .75)
    box(meeting, 'presentation_frame', (6.6,7.79,1.12), (2.2,.09,.65), 'metal')
    box(meeting, 'presentation_screen', (6.6,7.737,1.12), (2.08,.012,.53), 'screen', False)
    for i in range(4):
        box(meeting, f'screen_chart_{i}', (6.02+i*.34,7.727,1.01+i*.025),
            (.19,.008,.14+i*.05), 'teal' if i<3 else 'gold', False)
    for i,x in enumerate((5.85,7.35)):
        box(meeting, f'notebook_{i}', (x,5.45,.873), (.34,.26,.02), 'paper', False)
        box(meeting, f'pen_{i}', (x+.20,5.45,.875), (.018,.22,.018), 'gold', False)
    plant(meeting, 'meeting_plant', 9.25, 7.25)

    office = model(world, 'office_details')
    for i,x in enumerate((-7.8,-3.0)):
        monitor(office, f'computer_{i}', x, -5.65, .82)
        box(office, f'desk_pad_{i}', (x,-5.96,.867), (1.12,.36,.012), 'cap', False)
        box(office, f'files_{i}', (x+.70,-5.80,.95), (.24,.32,.18), 'paper')
    # Shelved books at the back stay outside the vehicle's established route.
    cabinet(office, 'back_storage', -8.2,-7.42,2.0,.55,'teal',.82)
    for i in range(9):
        box(office, f'book_{i}', (-8.9+i*.16,-7.42,1.04), (.11,.30,.25+(i%3)*.05),
            ('paper','blue','orange')[i%3])
    plant(office, 'office_plant', -1.0,-7.20)

    workshop = model(world, 'workshop_details')
    box(workshop, 'pegboard', (5,-7.80,1.06), (3.3,.065,.70), 'wood')
    for i in range(8):
        x = 3.65+i*.38
        box(workshop, f'tool_handle_{i}', (x,-7.75,1.03), (.045,.055,.28), 'metal')
        box(workshop, f'tool_head_{i}', (x,-7.75,1.20), (.17,.065,.08), 'gold' if i%2 else 'white')
    box(workshop, 'bench_mat', (5,-6.6,1.0), (1.15,.72,.014), 'cap', False)
    box(workshop, 'vise_base', (3.92,-6.75,1.06), (.34,.30,.15), 'metal')
    for i,x in enumerate((3.8,4.04)):
        box(workshop, f'vise_jaw_{i}', (x,-6.75,1.19), (.09,.28,.15), 'blue')
    cabinet(workshop, 'tool_drawers', 9.1,-2.8,.85,1.35,'orange',.86)
    for i in range(3):
        box(workshop, f'drawer_handle_{i}', (9.1,-3.49,.57+i*.13), (.58,.045,.025), 'metal', False)
    box(workshop, 'tool_case', (9.1,-2.8,1.03), (.65,.46,.21), 'metal')


def build():
    root = ET.Element('sdf', version='1.10')
    world = sub(root, 'world', name='obstacle_course')
    physics = sub(world, 'physics', name='default', type='ignored')
    sub(physics, 'max_step_size', '.001')
    sub(physics, 'real_time_factor', '1')
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'), ('imu', 'Imu')]:
        plugin = sub(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        if filename == 'sensors':
            sub(plugin, 'render_engine', 'ogre2')
    scene = sub(world, 'scene')
    sub(scene, 'ambient', '.53 .55 .57 1')
    sub(scene, 'background', '.20 .25 .29 1')
    sub(scene, 'shadows', 'true')
    sub(scene, 'grid', 'false')
    light = sub(world, 'light', name='studio_sun', type='directional')
    sub(light, 'pose', '0 0 12 0 0 0')
    sub(light, 'direction', '-.30 .40 -1')
    sub(light, 'diffuse', '.72 .69 .63 1')
    sub(light, 'specular', '.12 .12 .12 1')
    sub(light, 'cast_shadows', 'true')
    ground = model(world, 'ground')
    box(ground, 'floor', (0, 0, 0), (20, 16, .1), 'floor')
    # Lower the backing visual under the floor finishes to avoid depth/shadow acne.
    # The actual ground collision remains at z=0.05.
    ground.find('visual/pose').text = '0 0 -0.025 0 0 0'
    box(ground, 'presentation_plinth', (0, 0, -.13), (20.8, 16.8, .16), 'cap', False)
    room_floors = model(world, 'room_floor_finishes')
    # Three rooms north of the corridor and two rooms south of it.
    for number, x, y, sx, sy, color in [
        (1, -6.65, 4.7, 6.5, 6.45, '.68 .77 .72 1'),
        (2, 0, 4.7, 6.5, 6.45, '.74 .72 .65 1'),
        (3, 6.65, 4.7, 6.5, 6.45, '.51 .44 .34 1'),
        (4, -5, -4.7, 9.8, 6.45, '.51 .44 .34 1'),
        (5, 5, -4.7, 9.8, 6.45, '.68 .69 .68 1')]:
        box(room_floors, f'finish_{number}', (x, y, .052), (sx, sy, .003), color, False)
        floor_number(room_floors, number, x, 2.5 if y > 0 else -2.5)
    # Fine floor joints and a clear warm-white corridor with amber edge markings.
    for x in range(-9, 10):
        box(room_floors, f'joint_x_{x}', (x, 0, .055), (.008, 15.8, .002), '.61 .65 .62 1', False)
    for y in range(-7, 8):
        box(room_floors, f'joint_y_{y}', (0, y, .055), (19.8, .008, .002), '.61 .65 .62 1', False)
    box(room_floors, 'corridor', (0, 0, .058), (19.7, 2.6, .003), '.89 .88 .82 1', False)
    for y in (-1.17, 1.17):
        box(room_floors, f'corridor_edge_{y}', (0, y, .061), (19.5, .035, .003), 'gold', False)
    for i in range(-9, 10, 2):
        box(room_floors, f'center_dash_{i}', (i, 0, .062), (.5, .045, .003), '.61 .65 .64 1', False)
    walls = model(world, 'five_room_walls')
    wall(walls, 'north', 0, 8, 20, .2)
    wall(walls, 'south', 0, -8, 20, .2)
    wall(walls, 'east', 10, 0, .2, 16)
    wall(walls, 'west', -10, 0, .2, 16)
    for i, (lo, hi, door) in enumerate([(-10, -3.3, -6.6), (-3.3, 3.3, 0), (3.3, 10, 6.6)], 1):
        horizontal_with_door(walls, f'room_{i}_door', lo, hi, 1.4, door)
    for i, (lo, hi, door) in enumerate([(-10, 0, -5), (0, 10, 5)], 4):
        horizontal_with_door(walls, f'room_{i}_door', lo, hi, -1.4, door)
    wall(walls, 'north_divider_1', -3.3, 4.7, .18, 6.6)
    wall(walls, 'north_divider_2', 3.3, 4.7, .18, 6.6)
    wall(walls, 'south_divider', 0, -4.7, .18, 6.6)
    lab = model(world, 'room_1_laboratory')
    table(lab, 'lab_island', -6.6, 5.1, 2.2, 1.0, 'teal', .88)
    chair(lab, 'lab_chair_1', -7.25, 3.95, 'teal', -1)
    chair(lab, 'lab_chair_2', -5.95, 3.95, 'teal', -1)
    box(lab, 'bench_instrument', (-6.6, 5.1, 1.08), (.55, .45, .32), 'white')
    cabinet(lab, 'lab_storage', -8.9, 6.5, .8, 2.0, 'metal', 1.2)
    cabinet(lab, 'instrument', -4.3, 6.7, .8, .8, 'white', 1.0)
    storage = model(world, 'room_2_storage')
    cabinet(storage, 'rack_west', -2.2, 5.7, .75, 2.5, 'metal', 1.2)
    cabinet(storage, 'rack_east', 2.2, 5.7, .75, 2.5, 'metal', 1.2)
    for i, (x, y, yaw) in enumerate([(-.7, 6.8, .12), (.65, 6.8, -.10)]):
        cabinet(storage, f'crate_{i}', x, y, .85, .85, 'wood', .65, yaw)
    meeting = model(world, 'room_3_meeting')
    table(meeting, 'conference_table', 6.6, 5.5, 2.4, 1.15, 'wood', .82)
    for i, (x, y) in enumerate([(5.8, 4.35), (7.4, 4.35), (5.8, 6.65), (7.4, 6.65)]):
        chair(meeting, f'chair_{i}', x, y, 'blue', -1 if y < 5.5 else 1)
    box(meeting, 'table_display', (6.6, 5.5, 1.05), (.65, .12, .38), 'metal')
    office = model(world, 'room_4_office')
    for i, x in enumerate((-7.8, -3.0)):
        table(office, f'desk_{i}', x, -5.8, 2, .9, 'wood', .82)
        chair(office, f'chair_{i}', x, -4.6, 'blue', 1)
    cabinet(office, 'bookshelf', -8.9, -3.3, .65, 1.4, 'wood', 1.2)
    workshop = model(world, 'room_5_workshop')
    table(workshop, 'workbench', 5.0, -6.6, 3.0, 1.0, 'orange', .95)
    chair(workshop, 'workshop_chair', 5.0, -5.35, 'orange')
    open_shelf(workshop, 'tool_rack', 8.6, -5.7)
    cabinet(workshop, 'test_block', 2.0, -4.6, 1.0, 1.0, 'gold', .65, .3)
    floor_finishes(world)
    interior_details(world)
    ET.indent(root, space='  ')
    path = Path(__file__).with_name('five_rooms.sdf')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    print(path)


if __name__ == '__main__':
    build()
