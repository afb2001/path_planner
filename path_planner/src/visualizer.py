#!/usr/bin/env python

import colorsys
import pygame
from pygame.locals import *
import sys
import math
import numpy as np

Color_line = (0, 0, 0)
Color_line_middle = (180, 180, 180)
Color_line_path = (240, 0, 0)

Color_Red = (255, 0, 0)
Color_BLUE = (0, 0, 255)
Color_GREEN = (0, 255, 0)
Color_PURPLE = (255, 0, 255)
Color_CYAN = (0, 255, 255)

Color_Red_dark = (155, 0, 0)
Color_BLUE_dark = (0, 0, 155)
Color_GREEN_dark = (0, 155, 0)
Color_PURPLE_dark = (155, 0, 155)
Color_CYAN_dark = (0, 155, 155)

Color_BLACK = (0, 0, 0)
Color_WHITE = (255, 255, 255)
Color_GREY = (160, 160, 160)

color_base = 10
color_range = 240

colors = [Color_Red, Color_BLUE, Color_GREEN, Color_PURPLE, Color_CYAN, Color_Red_dark,
          Color_BLUE_dark, Color_GREEN_dark, Color_PURPLE_dark, Color_CYAN_dark]
theApp = 0

sprites = pygame.sprite.Group()


def dist_square(x1, y1, x2, y2):
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)


class DisplayItem:
    def __init__(self, x, y, h, cost, tag):
        self.x = x
        self.y = y
        self.h = h
        self.cost = cost
        self.tag = tag
        self.children = []  # trajectory, probably


class Iteration:
    def __init__(self, x, y, h):
        self.start = [x, y, h]
        self.samples = []
        self.cost_min = 0
        self.cost_max = 0
        self.items = []
        self.ribbons = []
        self.display_index = 0
        self.incumbent_f = 0
        self.contains_plan = False

    def get_color(self, cost):
        if cost > self.cost_max:
            return Color_BLACK
        if cost < self.cost_min:
            print ("bad cost: ", cost, self.cost_min, self.cost_max)
            return Color_BLACK
        r, g, b = colorsys.hsv_to_rgb(1 - ((cost - self.cost_min) / self.cost_max), 0.9, 0.75)
        return r * 255, g * 255, b * 255

    def append(self, item):
        if item.tag == "sample":
            self.samples.append([item.x, item.y])
        elif item.tag == "incumbent f":
            self.incumbent_f = item.cost
        elif item.tag == "dummy":
            if len(self.items) == 0 or self.items[-1].tag != "dummy":
                self.items.append(item)
            else:
                self.items[-1].children = []
        elif item.tag == "trajectory":
            # assume there's at least a dummy item (could probably check that)
            self.items[-1].children.append(item)
        elif item.tag == "vertex":
            if len(self.items) != 0 and self.items[-1].tag == "dummy":
                # overwrite dummy with vertex, keeping trajectory
                self.items[-1].x = item.x
                self.items[-1].y = item.y
                self.items[-1].h = item.h
                self.items[-1].cost = item.cost
                self.items[-1].tag = item.tag
            self.cost_min = min(self.cost_min, item.cost)
            self.cost_max = max(self.cost_max, item.cost)
        elif item.tag == "plan":
            self.contains_plan = True
            if len(self.items) == 0 or self.items[-1].tag != "plan":  # for now, stack plan in a plan item
                self.items.append(item)
            else:
                self.items[-1].children.append(item)
        else:
            self.items.append(item)

    def reset(self):
        self.display_index = 0

    def increment(self):
        # deciding to wrap around
        self.display_index += 1
        if self.display_index == len(self.items):
            self.display_index = 0

    def decrement(self):
        self.display_index -= 1
        if self.display_index < 0:
            self.display_index += len(self.items)

    def get_display_items(self):
        return self.items[0:self.display_index]


class Visualizer:
    def __init__(self, blocked, xlim, ylim):
        self.display = None
        self.screenH = 1400
        self.screenW = 1200
        self.maxX = xlim
        self.maxY = ylim
        self.xLim = xlim
        self.yLim = ylim
        self.lines = 9
        self.originX = 0
        self.originY = 0
        self.moveX = 0
        self.moveY = 0

        self.triangleX = (0, -5, 5)
        self.triangleY = (-10, 10, 10)

        self.static_obs = blocked

        self.draw_trajectories = True
        self.draw_samples = True
        self.draw_ribbons = True
        self.draw_vertices = True
        self.draw_vertex_costs = True

        self.iterations = []
        self.iteration_index = 0

        # declare stuff defined in on_init
        self.w, self.h, self.scaleW, self.scaleH, self.startW, self.startH = 0, 0, 0, 0, 0, 0
        self.curr_x, self.curr_y, self.start_heading, self.index = 0, 0, 0, 0
        self.background = None
        pygame.font.init()
        try:
            self.font = pygame.font.Font("r.ttf", 15)
            self.font1 = pygame.font.Font("r.ttf", 8)
        except:
            self.font = pygame.font.SysFont(None, 20)
            self.font1 = pygame.font.SysFont(None, 10)

    def on_init(self):
        pygame.init()
        self.display = pygame.display.set_mode(
            (self.screenH, self.screenW), HWSURFACE | RESIZABLE)
        self.w, self.h = pygame.display.get_surface().get_size()
        self.scaleW = self.w * 0.8
        self.scaleH = self.h * 0.8
        self.startW = self.w * 0.1
        self.startH = self.h * 0.1
        self.background = pygame.Surface(self.display.get_size())
        self.background.fill((240, 240, 240))
        self.background = self.background.convert()
        self.curr_x = 0
        self.curr_y = 0
        self.start_heading = 0
        self.index = 0

    def on_event(self, event):
        if event.type == QUIT:
            pygame.quit()
            exit(0)
        elif event.type == pygame.KEYDOWN:
            # pan the view
            if event.key == pygame.K_LEFT:
                self.originX -= self.scaleW / (self.lines + 1)
                self.moveX += self.maxX / (self.lines + 1)
            elif event.key == pygame.K_RIGHT:
                self.originX += self.scaleW / (self.lines + 1)
                self.moveX -= self.maxX / (self.lines + 1)
            elif event.key == pygame.K_DOWN:
                self.originY += self.scaleH / (self.lines + 1)
                self.moveY += self.maxY / (self.lines + 1)
            elif event.key == pygame.K_UP:
                self.originY -= self.scaleH / (self.lines + 1)
                self.moveY -= self.maxY / (self.lines + 1)

            # scale the view
            elif event.key == pygame.K_MINUS:
                d = 1 if self.maxX < 100 and self.maxY < 100 else 10
                p = d * 10
                x, y = self.scale_item(self.curr_x, self.curr_y)
                x = int(round(((x - self.startW) / self.scaleW) * 10, 5))
                y = int(round(((1 - (y - self.startH) / self.scaleH) * 10), 5))
                self.maxX += p
                self.maxY += p
                self.originY = self.scaleH * ((int(self.curr_y) / d) * d / float(self.maxY)) - y * self.scaleH / (self.lines + 1)
                self.originX = -self.scaleW * ((int(self.curr_x) / d) * d / float(self.maxX)) + x * self.scaleW / (self.lines + 1)
                cy = (int(self.curr_y) / d) * d
                cx = (int(self.curr_x) / d) * d
                self.moveY = cy - y * self.maxY / (self.lines + 1)
                self.moveX = cx - x * self.maxX / (self.lines + 1)
            elif event.key == pygame.K_EQUALS:
                if self.maxX >= 20 and self.maxY >= 20:
                    d = 10 if self.maxX >= 200 and self.maxY >= 200 else 1
                    p = d * 10
                    x, y = self.scale_item(self.curr_x, self.curr_y)
                    x = int(round(((x - self.startW) / self.scaleW) * 10, 5))
                    y = int(round(((1 - (y - self.startH) / self.scaleH) * 10), 5))
                    self.maxX -= p
                    self.maxY -= p
                    self.originY = self.scaleH * ((int(self.curr_y) / d) * d / float(self.maxY)) - y * self.scaleH / (self.lines + 1)
                    self.originX = -self.scaleW * ((int(self.curr_x) / d) * d / float(self.maxX)) + x * self.scaleW / (self.lines + 1)
                    cy = (int(self.curr_y) / d) * d
                    cx = (int(self.curr_x) / d) * d
                    self.moveY = cy - y * self.maxY / (self.lines + 1)
                    self.moveX = cx - x * self.maxX / (self.lines + 1)

            # reset the view
            elif event.key == pygame.K_SPACE:
                d = 10 if self.maxX >= 100 and self.maxY >= 100 else 1
                self.maxY = self.yLim
                self.maxX = self.xLim
                self.originY = self.scaleH * ((int(self.curr_y) / d) * d / float(self.maxY))
                self.originX = -self.scaleW * ((int(self.curr_x) / d) * d / float(self.maxX))
                self.moveY = (int(self.curr_y) / d) * d
                self.moveX = (int(self.curr_x) / d) * d

            elif event.key == pygame.K_ESCAPE:
                # exit alias (convenience)
                pygame.quit()
                exit(0)

            # visualization-related events
            elif event.key == pygame.K_n:
                # show the next vis item
                self.iterations[self.iteration_index].increment()
            elif event.key == pygame.K_BACKSPACE or event.key == pygame.K_b:
                # show the previous vis item
                self.iterations[self.iteration_index].decrement()
            elif event.key == pygame.K_j:
                # jump to the previous planning iteration
                # shift means jump to previous goal found
                self.decrement_iteration(pygame.key.get_mods() & pygame.KMOD_SHIFT)
            elif event.key == pygame.K_k:
                # jump to the next planning iteration
                # shift means jump to next goal found
                self.increment_iteration(pygame.key.get_mods() & pygame.KMOD_SHIFT)
            elif event.key == pygame.K_t:
                # toggle showing trajectories
                self.draw_trajectories = not self.draw_trajectories
            elif event.key == pygame.K_s:
                # toggle showing samples
                self.draw_samples = not self.draw_samples
            elif event.key == pygame.K_r:
                # toggle showing ribbons
                self.draw_ribbons = not self.draw_ribbons
            elif event.key == pygame.K_v:
                # toggle showing vertices (why not?)
                self.draw_vertices = not self.draw_vertices
            elif event.key == pygame.K_c:
                # toggle showing vertex costs (again, why not)
                self.draw_vertex_costs = not self.draw_vertex_costs

    def increment_iteration(self, jump_to_goal):
        self.iterations[self.iteration_index].reset()
        self.iteration_index += 1
        if self.iteration_index >= len(self.iterations):
            self.iteration_index -= 1
        else:
            if jump_to_goal and not self.iterations[self.iteration_index].contains_plan:
                # keep going until we hit a goal
                self.increment_iteration(jump_to_goal)

    def decrement_iteration(self, jump_to_goal):
        self.iterations[self.iteration_index].reset()
        if self.iteration_index > 0:
            self.iteration_index -= 1
            if jump_to_goal and not self.iterations[self.iteration_index].contains_plan:
                # keep going until we hit a goal
                self.decrement_iteration(jump_to_goal)

    def on_render(self):
        self.display.blit(self.background, (0, 0))
        self.draw_line()
        self.draw_text()
        self.draw()
        self.draw_incumbent_f()
        self.draw_found_goal_text()
        sprites.draw(self.display)
        sprites.update()
        pygame.display.flip()

    def draw_line(self):
        pygame.draw.line(self.display, Color_line, (self.startW, self.startH), (self.startW + self.scaleW, self.startH))
        pygame.draw.line(self.display, Color_line, (self.startW, self.startH), (self.startW, self.startH + self.scaleH))
        pygame.draw.line(self.display, Color_line, (self.startW, self.startH + self.scaleH), (self.startW + self.scaleW, self.startH + self.scaleH))
        pygame.draw.line(self.display, Color_line, (self.startW + self.scaleW, self.startH), (self.startW + self.scaleW, self.startH + self.scaleH))
        scaleh = self.scaleH / (self.lines + 1)
        scalew = self.scaleW / (self.lines + 1)
        for i in range(1, self.lines + 1):
            pygame.draw.line(self.display, Color_line_middle, self.scale_xy(0, i * scaleh), self.scale_xy(self.scaleW, i * scaleh))
            pygame.draw.line(self.display, Color_line_middle, self.scale_xy(i * scalew, 0), self.scale_xy(i * scalew, self.scaleH))

    def get_color(self, cost):
        return self.iterations[self.iteration_index].get_color(cost)

    def draw(self):
        for obs in self.static_obs:
            self.draw_static_obs(Color_BLACK, *obs)
        current_it = self.iterations[self.iteration_index]
        self.draw_vehicle(current_it.start[2], Color_BLUE, *self.scale_item(current_it.start[0], current_it.start[1]))
        if self.draw_samples:
            for sample in current_it.samples:
                self.draw_circle(Color_GREY, *self.scale_item(sample[0], sample[1]))
        if self.draw_ribbons:
            for ribbon in current_it.ribbons:
                self.draw_ribbon(ribbon)
        for item in current_it.get_display_items():
            self.draw_item(item)

    def draw_item(self, item):
        if item.tag == "vertex":
            if self.draw_vertices:
                yy = self.draw_vehicle(item.h, self.get_color(item.cost), *self.scale_item(item.x, item.y))
                self.draw_text_cost(item.cost, self.scale_item(item.x, item.y)[0], yy)
        elif item.tag == "lastPlanEnd":
            yy = self.draw_vehicle(item.h, Color_BLACK, *self.scale_item(item.x, item.y))
            self.draw_text_cost(item.cost, self.scale_item(item.x, item.y)[0], yy)
        elif item.tag == "plan":
            self.draw_circle(Color_BLUE, *self.scale_item(item.x, item.y))
        elif item.tag == "goal":
            self.draw_circle(Color_GREEN_dark, *self.scale_item(item.x, item.y))
        elif item.tag == "dummy":
            # TODO! -- figure out why dummy vertices aren't getting overwritten
            # since it's a dummy there was no vertex to end the trajectory, so don't draw the children
            return
        else:
            # not sure what this would be but we'll just draw a circle I guess?
            print ("Unknown tag (" + item.tag + ") getting drawn as a circle with cost: " + str(item.cost))
            self.draw_circle(self.get_color(item.cost), *self.scale_item(item.x, item.y))
        for child in item.children:
            if item.tag == "plan":
                self.draw_circle(Color_BLUE, *self.scale_item(child.x, child.y))
            elif self.draw_trajectories:  # assume they're trajectories at this point
                self.draw_circle(self.get_color(item.cost), *self.scale_item(child.x, child.y))

    def draw_static_obs(self, color, x, y):
        pygame.draw.polygon(self.display, color, (self.scale_item(x, y), self.scale_item(
            x + 1, y), self.scale_item(x + 1, y + 1), self.scale_item(x, y + 1)))

    def draw_dot(self, color, x, y):
        pygame.draw.polygon(self.display, color, (self.scale_item(x, y), self.scale_item(
            x + 0.3, y), self.scale_item(x + 0.3, y + 0.3), self.scale_item(x, y + 0.3)))

    def draw_vehicle(self, angle, color, x, y):
        t_x = []
        t_y = []
        c = math.cos(angle)
        s = math.sin(angle)
        for i in range(3):
            t_x.append(self.triangleX[i] * c - self.triangleY[i] * s + x)
            t_y.append(self.triangleX[i] * s + self.triangleY[i] * c + y)
        pygame.draw.polygon(self.display, color, ((t_x[0], t_y[0]), (t_x[1], t_y[1]), (t_x[2], t_y[2])))
        return min(t_y[0], t_y[1], t_y[2])

    def draw_circle(self, color, x, y):
        pygame.draw.circle(self.display, color, (int(x), int(y)), 2)

    def draw_ribbon(self, ribbon):
        pygame.draw.line(self.display, Color_Red, self.scale_item(ribbon[0], ribbon[1]),
                         self.scale_item(ribbon[2], ribbon[3]), 5)

    def draw_text_cost(self, cost, x, y):
        if not self.draw_vertex_costs:
            return
        text = str(int(cost))
        text_surface = self.font.render(text, True, (0, 0, 0))
        y += text_surface.get_height()
        self.display.blit(text_surface, (x, y))

    def draw_incumbent_f(self):
        text = "Incumbent f-value: " + str(self.iterations[self.iteration_index].incumbent_f)
        text_surface = self.font.render(text, True, (0, 0, 0))
        self.display.blit(text_surface, (10, 10))

    def draw_found_goal_text(self):
        text = "No better goal exists this iteration"
        for item in self.iterations[self.iteration_index].items:
            if item.tag == "goal":
                text = "Found goal this iteration with f-value " + str(item.cost)
        text_surface = self.font.render(text, True, (0, 0, 0))
        self.display.blit(text_surface, (200, 10))

    def draw_text(self):
        scale_h = self.scaleH / (self.lines + 1)
        scale_w = self.scaleW / (self.lines + 1)
        for i in range(0, self.lines + 2):
            text = str(int(self.moveY + (self.lines - i + 1)
                           * self.maxY / (self.lines + 1)))
            text_surface = self.font.render(text, True, (0, 0, 0))
            x, y = self.scale_xy(0, (i) * scale_h)
            x -= text_surface.get_width() + 10
            y -= text_surface.get_height() / 2
            self.display.blit(text_surface, (x, y))
            text = str(int(self.moveX + (self.lines - i + 1)
                           * self.maxX / (self.lines + 1)))
            text_surface = self.font.render(text, True, (0, 0, 0))
            x, y = self.scale_xy((self.lines - i + 1) * scale_w, self.scaleH)
            x -= text_surface.get_width() / 2
            y += 10
            self.display.blit(text_surface, (x, y))

    def update(self):
        event = pygame.event.wait()
        self.on_event(event)
        self.on_render()

    def scale_xy(self, x, y):
        return self.startW + x, self.startH + y

    def scale_item(self, x, y):
        return self.originX + self.startW + self.scaleW * (
                x / float(self.maxX)), self.originY + self.startH + self.scaleH - self.scaleH * (
                       y / float(self.maxY))

    def scale_path(self, x, y):
        return self.originX + self.startW + self.scaleW * (
                x / float(self.maxX)), self.originY + self.startH + self.scaleH - self.scaleH * (
                       y / float(self.maxY))

    def load(self, file_name):
        with open(file_name, "r") as input_file:
            input_lines = input_file.readlines()
        pygame.display.set_caption('Visualizing ' + file_name)
        self.iterations = []
        self.iteration_index = 0

        # Ribbons need special treatment because they look different than vertices
        adding_ribbons = False

        for line in input_lines:
            for c in "():,\n":  # drop extra characters
                line = line.replace(c, "")
            line = line.split(' ')

            # ribbon-related bits
            if line[0] == "End" and line[1] == "Ribbons":
                adding_ribbons = False
            elif line[0] == "Ribbons":
                adding_ribbons = True
            elif adding_ribbons:
                # ribbon mode means input will look different, hence the flag
                self.iterations[-1].ribbons.append([float(line[0]), float(line[1]), float(line[3]), float(line[4])])

            elif line[0] == "Trajectory":
                if len(self.iterations) != 0:  # make sure there's been a start already
                    self.iterations[-1].append(DisplayItem(0, 0, 0, 0, "dummy"))  # append a dummy item to be replaced
            elif line[0] == "Incumbent" and line[1] == "f-value":
                if len(self.iterations) != 0:  # make sure there's been a start already
                    self.iterations[-1].append(DisplayItem(0, 0, 0, float(line[2]), "incumbent f"))
            else:
                x = (float(line[1]))
                y = (float(line[2]))
                h = (float(line[3]))
                cost = (float(line[9]))
                heuristic = (float(line[11]))
                tag = line[12].strip().lower()
                if tag == "start":
                    self.iterations.append(Iteration(x, y, h))
                else:
                    if len(self.iterations) == 0:
                        continue
                    self.iterations[-1].append(DisplayItem(x, y, h, cost + heuristic, tag))


def dist(x, x1, y, y1):
    return (x - x1) ** 2 + (y - y1) ** 2


if __name__ == "__main__":

    map_file_name = input_file_name = None
    if len(sys.argv) == 3:
        map_file_name = sys.argv[1]
        input_file_name = sys.argv[2]
    elif len(sys.argv) == 2:
        input_file_name = sys.argv[1]
    else:
        print 'Usage: "./visualizer.py map_file input_file or"\n' \
            '"./visualizer.py input_file"'
        exit(0)

    static_obs = []
    # load map
    if map_file_name:
        with open(map_file_name, "r") as map_file:
            map_contents = map_file.readlines()
        # max_x = np.abs(int(int(map_contents[1])))
        # max_y = np.abs(int(int(map_contents[2])))
        max_x = max(len(x) for x in map_contents)
        max_y = len(map_contents)
        for i in range(1, len(map_contents)):
            for j in range(0, len(map_contents[i])):
                if map_contents[i][j] == '#':
                    static_obs.append((j, (max_y - i + 1)))
    else:
        max_x = 100
        max_y = 100

    theApp = Visualizer(static_obs, max_x, max_y)
    theApp.on_init()
    theApp.load(input_file_name)

    try:
        while True:
            theApp.update()
    except Exception as e:
        print "ERROR"
        print(e)
        raise
    # finally:
    #     print "TERMINATE"
    #     theApp.stop()
    #     exit(0)
