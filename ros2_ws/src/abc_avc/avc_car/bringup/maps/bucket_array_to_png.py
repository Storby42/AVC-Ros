import pygame as pg
import numpy as np

feet_to_meters=0.3048

radius = 0.5

width = 100
height = 80
map_conversion=20*feet_to_meters

# starts at (0,0) bottom left with frame of width 100, height 80
# indexes through buckets from bottom right -> clockwise
buckets=np.array([[80,20], [50,25],[50,35],[20,20],[20,60],[50,57.5],[50,62.5],[80,60]])
ramp=[16,40]

background = (255, 255, 255)
black = (0, 0, 0)

win = pg.display.set_mode((width*map_conversion, height*map_conversion))
win.fill(background)

width = 0

for bucket in buckets:
    pg.draw.circle(win, black, bucket*map_conversion, radius*map_conversion, width)

ramp_draw=pg.Rect(ramp[0]*map_conversion,ramp[1]*map_conversion,4*map_conversion,4*map_conversion)
pg.draw.rect(win, black, ramp_draw, width)

fname = "map.png"
win = pg.transform.flip(win,False,True) # surface, flip_x, flip_y - note that this only changes the file, not the display
pg.image.save(win, fname)
print("file {} has been saved".format(fname))

"""
# update the display window to show the drawing
pg.display.flip()

# (press escape key or click window title bar x to exit)
while True:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            # most reliable exit on x click
            pg.quit()
            raise SystemExit
        elif event.type == pg.KEYDOWN:
            # optional exit with escape key
            if event.key == pg.K_ESCAPE:
                pg.quit()
                raise SystemExit

"""


