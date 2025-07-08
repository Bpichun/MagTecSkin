#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Oct 2024

@author: ariel
"""

import sys
import numpy as np
import pygame


sys.path.append('../')

# import Constants
cell_size = 54
cell_gap = 55 - cell_size
# grid_rows = Constants.grid_rows
# grid_cols = Constants.grid_cols
grid_rows = 10
grid_cols = 10

window_width = grid_cols * (cell_size + cell_gap) + cell_gap
window_height = grid_rows * (cell_size + cell_gap) + cell_gap
print(window_width, window_height)
ratio = 75
mask = []
points = np.load("PointsOnSurface.npy")

# print(f"initial points: {points}")
image_ = False
'''==========================================
                Grid Creation Functions
   ========================================'''
def read_mask(filename):
    global mask
    with open(filename, "r") as f:
        for line in f:
            row = list(map(int, line.split()))
            mask.append(row)

def update_points():  ## reading current points on sofa
    global points
    try:
        points = np.load("../CuttingPoints/PointsOnSurfaceUpdated.npy")
    except Exception:
        points = np.load("../CuttingPoints/PointsOnSurface.npy")

def draw_grid(screen):
    valid_cells = []
    for row in range(grid_rows):
        for col in range(grid_cols):
            x = col * (cell_size + cell_gap) + cell_gap
            y = row * (cell_size + cell_gap) + cell_gap
            if mask[row][col] == 1:
                color = (0, 0, 255)
                if image_:
                    new_x = x + (cell_size - 20) // 2
                    new_y = y + (cell_size - 20) // 2
                    pygame.draw.rect(screen, color, (new_x, new_y, 20, 20))
                else:
                    pygame.draw.rect(screen, color, (x, y,cell_size, cell_size))
            else:
                color = (169, 169, 169)
                if image_ == False:
                    pygame.draw.rect(screen, color, (x, y, cell_size, cell_size))

            coordinates = [x,y,cell_size]
            valid_cells.append(coordinates)

def normalize_coordinates(x, y):
    grid_width = grid_cols * (cell_size + cell_gap)
    grid_height = grid_rows * (cell_size + cell_gap)
    x_normalized = (x / grid_width) * grid_cols
    y_normalized = (y / grid_height) * grid_rows
    return x_normalized, y_normalized

def denormalize_coordinates(x_norm, y_norm):
    grid_width = grid_cols * (cell_size + cell_gap)
    grid_height = grid_rows * (cell_size + cell_gap)
    x_real = (x_norm / grid_cols) * grid_width
    y_real = (y_norm / grid_rows) * grid_height
    return x_real, y_real

def draw_circle(screen, color=(255, 255, 0)):
    x,y = normalize_coordinates(global_x, global_y)
    print(f"x: {x} y: {y}")
    x,y = denormalize_coordinates(x, y)
    print(f"x: {x} y: {y}")
    pygame.draw.circle(screen, color, (global_x, global_y), ratio, 5)

def image(show):
    global image_
    
    bg = pygame.image.load("../foss.png")
    bg = pygame.transform.scale(bg, (screen.get_width(), screen.get_height()+50))
    ship_top = screen.get_height() - bg.get_height()+25
    ship_left = screen.get_width()/2 - bg.get_width()/2 -5

    if show == True:
        screen.blit(bg, (ship_left,ship_top))
        image_ = True
    else:
        pass

'''==========================================
                 Event Mouse Function
   ========================================'''

def mouse_move_event(row, col):
    global local_x, local_y, global_x, global_y
    position = points[row * grid_cols + col]
    print(f"row: {row} col: {col} position: {position}")
    mouse_x, mouse_y = pygame.mouse.get_pos()
    print([mouse_x, mouse_y])

    #10,6 
    global_x, global_y = mouse_x, mouse_y
    local_x = mouse_x - (col * (cell_size + cell_gap) + cell_gap)
    local_y = mouse_y - (row * (cell_size + cell_gap) + cell_gap)
    center_x = col * (cell_size + cell_gap) + cell_gap + cell_size / 2
    center_y = row * (cell_size + cell_gap) + cell_gap + cell_size / 2
    # print(f"local mouse position in cell: ({local_x}, {local_y})")
    # print(f"global mouse position  x: {mouse_x} y: {mouse_y}")


'''==========================================
            Neighbors Functions
   ========================================'''

def getNeighborIdxs(row,col):
    cells = []
    # cell size
    for x in range(grid_rows):
        for y in range(grid_cols):
            cell_top_left_x = y * (cell_size + cell_gap) + cell_gap
            cell_top_left_y = x * (cell_size + cell_gap) + cell_gap
            cell_bottom_right_x = cell_top_left_x + cell_size
            cell_bottom_right_y = cell_top_left_y + cell_size
            # calculate distances
            distance_x =  abs(global_x - (cell_top_left_x + cell_size / 2)) - cell_size / 2
            distance_y =  abs(global_y - (cell_top_left_y + cell_size / 2)) - cell_size / 2
            # if distance to quad is less or equal to quad ratio, intersection = true
            if (distance_x ** 2 + distance_y ** 2 <= ratio ** 2):
                cells.append((x, y))
    return cells



def getNeighborsValues():
    global center_x, center_y
    neighbors = getNeighborIdxs(row,col)
    intensityList = []
    centersList = []
    activeNeighbors = []
    for neighbor in neighbors:
        neighbor_row, neighbor_col = neighbor
        # read if neighbor is valid
        if mask[neighbor_row][neighbor_col] == 1:
            # append active neighbors correctly
            activeNeighbors.append((neighbor_row, neighbor_col))
            # calculate center of neighbor
            x_pos_neighbor = neighbor_col * (cell_size + cell_gap) + cell_gap
            y_pos_neighbor = neighbor_row * (cell_size + cell_gap) + cell_gap
            cell_center_x = x_pos_neighbor + cell_size / 2
            cell_center_y = y_pos_neighbor + cell_size / 2
            # calculate euclidean distance
            distance = ((cell_center_x - global_x) ** 2 + (cell_center_y - global_y) ** 2) ** 0.5

            
            if distance == 0:
                intensity = 1
            else:
                intensity = 1/distance


            color_intensity = min(int(intensity * 5000),255)
            # print(f"intensity neighbor {neighbor_row, neighbor_col}: {intensity}")
            color = (color_intensity, color_intensity, color_intensity)
            pygame.draw.rect(screen, color, (x_pos_neighbor, y_pos_neighbor, cell_size, cell_size))
            intensityList.append(intensity)
            center = [cell_center_x, cell_center_y]
            centersList.append(center)
            if intensity == max(intensityList):
                center_x = cell_center_x
                center_y = cell_center_y
    # print(f"activeNeighbors: {activeNeighbors}")
    return intensityList, activeNeighbors, centersList

def fileIdxsAndWeights(active):
    if active == True:
        intensityList, neighbors, centersList = getNeighborsValues()
        # print(f"neighbors: {neighbors}")
        with open('IdxList.txt', 'w') as f:
            for neighbor in neighbors:
                f.write(f"{neighbor[0]} {neighbor[1]}\n")
                # print(f"IdxList: {neighbor[0]} {neighbor[1]}\n")

        with open('WeightList.txt', 'w') as f:
            for i in intensityList:
                f.write(f"{i}\n")
                # print(f"WeightList:  {i}\n")

    else:
        with open('IdxList.txt', 'w') as f:
            f.write(f"")

        with open('WeightList.txt', 'w') as f:
            f.write(f"")

def getWeightedAverageSim():
    Gwx, Gwy, Gwz = 0, 0, 0
    intensityList, neighbors, centersList = getNeighborsValues()
    print(f"centersList: {centersList}")
    
    # print(f"neighbors to sofaposition: {neighbors}")
    Vsum = np.sum(intensityList)
    try:
        for ij in range(len(intensityList)):
            Wij = intensityList[ij]/Vsum
            Gwx += (Wij * centersList[ij][0])
            Gwy += (Wij * centersList[ij][1])
        print(f"Gwx: {Gwx} Gwy: {Gwy}")
    except:
        pass

    pygame.draw.circle(screen, (255,255,0), (Gwx,Gwy),  10)

'''==========================================
                    MAIN
   ========================================'''



def main():
    global mask, screen, row,col
    show_image = False
    pygame.init()
    screen = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption(f"{grid_rows}x{grid_cols} grid with mouse tracking")
    read_mask("Mask.txt")
   
    clock = pygame.time.Clock()
    running = True
    screen.fill((255, 255, 255))
    # image(show_image)


    draw_grid(screen)
    pygame.display.flip()
    click = False  
    touch = False
    while running:
        for event in pygame.event.get():
            # print(f"event:{event}")
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    click = True
                else:
                    click = False
            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    click = False  

            if event.type == pygame.FINGERDOWN:
                touch = True
            if event.type == pygame.FINGERUP:
                touch = False

            try:
                if click:
                    mouse_x, mouse_y = event.pos
                    col = (mouse_x - cell_gap) // (cell_size + cell_gap)
                    row = (mouse_y - cell_gap) // (cell_size + cell_gap)
                    # print(f"col main: {col} row main: {row}")
                    if 0 <= row < grid_rows and 0 <= col < grid_cols and mask[row][col] == 1:
                        screen.fill((255,255, 255))
                        # image(show_image)
                        draw_grid(screen)
                        x_pos_cell = col * (cell_size + cell_gap) + cell_gap
                        y_pos_cell = row * (cell_size + cell_gap) + cell_gap
                        # print(f"x pos cell: {x_pos_cell} y pos cell: {y_pos_cell}")
                        mouse_move_event(row, col)
                        draw_circle(screen, 100)
                        fileIdxsAndWeights(True) # this only is for Sofa
                        getWeightedAverageSim() # this only is for pygame
                        pygame.display.flip()
                       
                    else:
                        click = False


                elif touch:
                    mouse_x, mouse_y = event.x, event.y
                    col = (mouse_x - cell_gap) // (cell_size + cell_gap)
                    row = (mouse_y - cell_gap) // (cell_size + cell_gap)
                    # print(f"col main: {col} row main: {row}")
                    if 0 <= row < grid_rows and 0 <= col < grid_cols and mask[row][col] == 1:
                        screen.fill((255,255, 255))
                        # image(show_image)
                        draw_grid(screen)
                        x_pos_cell = col * (cell_size + cell_gap) + cell_gap
                        y_pos_cell = row * (cell_size + cell_gap) + cell_gap
                        # print(f"x pos cell: {x_pos_cell} y pos cell: {y_pos_cell}")
                        mouse_move_event(row, col)
                        draw_circle(screen, 100)
                        print("globals: ",global_x, global_y)
                        fileIdxsAndWeights(True) # this only is for Sofa
                        getWeightedAverageSim() # this only is for pygame
                        pygame.display.flip()
                       
                    else:
                        touch = False
                else:
                    screen.fill((255, 255, 255))
                    # image(show_image)
                    draw_grid(screen)
                    pygame.display.flip()
                    fileIdxsAndWeights(False)
                    # print(f"No click")
            except:
                pass
           


    update_points()
    clock.tick(30)
       
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()

