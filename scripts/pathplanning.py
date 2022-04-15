#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import numpy as np
import random


class Bounds:
	def __init__(self, x_min, x_max, y_min, y_max, step_size, inflation):
		self.x_min = x_min
		self.x_max = x_max
		self.y_min = y_min
		self.y_max = y_max
		self.step_size = step_size
		self.inflation = inflation


class Node:
	def __init__(self, x, y, cost, parent_index):
		self.x = x
		self.y = y
		self.cost = cost
		self.parent_index = parent_index


def index_func(x, y, bounds):
	return ((y - bounds.y_min) / bounds.step_size) * \
			((bounds.x_max - bounds.x_min + bounds.step_size) / bounds.step_size)\
			+ ((x - bounds.x_min) / bounds.step_size)


def distance(point1, point2):
	d = np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
	return d


#returns false if the node is not valid, true if it is.
def check_valid_node(xy, obstacle_list, bounds):
	valid_node = True
	for point in obstacle_list:
		if xy[0] == point[0] and xy[1] == point[1]:
			valid_node = False
		elif distance(xy, point) <= bounds.inflation:
			valid_node = False
	if not bounds.x_min + bounds.inflation <= xy[0] <= bounds.x_max - bounds.inflation or not bounds.y_min\
		+ bounds.inflation <= xy[1] <= bounds.y_max - bounds.inflation:
		valid_node = False
	return valid_node


def check_valid_node_rrt(xy, obstacle_list, bounds):
	valid_node = True
	for point in obstacle_list:
		if point[0] - bounds.inflation <= xy[0] <= point[0] + bounds.inflation and point[1] - bounds.inflation <= xy[1] <= point[1] + bounds.inflation:
			valid_node = False
		if xy[0] == point[0] and xy[1] == point[1]:
			valid_node = False
	if not bounds.x_min + bounds.inflation <= xy[0] <= bounds.x_max - bounds.inflation or not bounds.y_min\
		+ bounds.inflation <= xy[1] <= bounds.y_max - bounds.inflation:
		valid_node = False
	return valid_node


#returns the index of neighbor nodes.
def get_neighbors(node, bounds):
	neighbors = []
	modifiers = (-bounds.step_size, 0, bounds.step_size)
	for x in modifiers:
		for y in modifiers:
			if x == 0 and y == 0:
				pass
			elif bounds.x_min <= node.x + x <= bounds.x_max and bounds.y_min <= node.y + y <= bounds.y_max :
				neighbors.append((node.x + x, node.y + y))
	return neighbors


def dijkstras(start_node, end_node, obstacle_list, bounds):
	visited = dict()
	unvisited = dict()
	current_node = start_node
	unvisited[index_func(current_node.x, current_node.y, bounds)] = current_node
	while len(unvisited) != 0:
		current_index = min(unvisited, key=lambda x: unvisited[x].cost)
		current_node = unvisited[current_index]
		neighbors = get_neighbors(current_node, bounds)
		for neighbor in neighbors:
			if check_valid_node(neighbor, obstacle_list, bounds):
				cost = current_node.cost + distance((current_node.x, current_node.y), neighbor)
				index = index_func(neighbor[0], neighbor[1], bounds)
				if index in unvisited:
					if cost < unvisited[index].cost:
						unvisited[index].cost = cost
						unvisited[index].parent_index = current_index
				elif index in visited:
					if cost < visited[index].cost:
						visited[index].cost = cost
						visited[index].parent_index = current_index
				elif index not in visited and index not in unvisited:
					unvisited[index] = Node(neighbor[0], neighbor[1], cost, current_index)
		visited[current_index] = current_node
		unvisited.pop(current_index)
	shortest_path_index = visited[index_func(end_node.x, end_node.y, bounds)].parent_index
	shortest_path = [(end_node.x, end_node.y)]
	while shortest_path_index != -1:
		shortest_path.append((visited[shortest_path_index].x, visited[shortest_path_index].y))
		shortest_path_index = visited[shortest_path_index].parent_index
	end_cost = 0
	for i in range(len(shortest_path)):
		if i != len(shortest_path) - 1:
			end_cost += distance(shortest_path[i], shortest_path[i + 1])

	return shortest_path, end_cost


def a_star(start_node, end_node, obstacle_list, bounds):
	visited = dict()
	unvisited = dict()
	current_node = start_node
	unvisited[index_func(current_node.x, current_node.y, bounds)] = current_node
	while len(unvisited) != 0:
		current_index = min(unvisited, key=lambda x: unvisited[x].cost)
		current_node = unvisited[current_index]
		neighbors = get_neighbors(current_node, bounds)
		for neighbor in neighbors:
			if check_valid_node(neighbor, obstacle_list, bounds):
				cost = current_node.cost + distance((current_node.x, current_node.y), neighbor) + distance((current_node.x, current_node.y), (end_node.x, end_node.y))
				index = index_func(neighbor[0], neighbor[1], bounds)
				if index in unvisited:
					if cost < unvisited[index].cost:
						unvisited[index].cost = cost
						unvisited[index].parent_index = current_index
				elif index in visited:
					if cost < visited[index].cost:
						visited[index].cost = cost
						visited[index].parent_index = current_index
				elif index not in visited and index not in unvisited:
					unvisited[index] = Node(neighbor[0], neighbor[1], cost, current_index)
		visited[current_index] = current_node
		unvisited.pop(current_index)
		if current_node.x == end_node.x and current_node.y == end_node.y:
			break
	shortest_path_index = visited[index_func(end_node.x, end_node.y, bounds)].parent_index
	shortest_path = [(end_node.x, end_node.y)]
	while shortest_path_index != -1:
		shortest_path.append((visited[shortest_path_index].x, visited[shortest_path_index].y))
		shortest_path_index = visited[shortest_path_index].parent_index
	end_cost = 0
	for i in range(len(shortest_path)):
		if i != len(shortest_path) - 1:
			end_cost += distance(shortest_path[i], shortest_path[i+1])

	return shortest_path, end_cost


def rrt(start_node, end_node, obstacle_list, bounds, step):

	current_node = start_node
	nodes = [current_node]

	while distance((current_node.x, current_node.y), (end_node.x, end_node.y)) > 0.1:
		rand_x = random.uniform(bounds.x_min, bounds.x_max)
		rand_y = random.uniform(bounds.y_min, bounds.y_max)
		closest_dist = float('inf')
		closest_node = current_node

		for node in nodes:
			dist = distance((rand_x, rand_y), (node.x, node.y))
			if dist < closest_dist:
				closest_dist = dist
				closest_node = node
		direction = (rand_x - current_node.x, rand_y - current_node.y)
		direction_magnitude = distance((rand_x, rand_y), (current_node.x, current_node.y))
		normalized_direction = ((direction[0] / direction_magnitude), (direction[1] / direction_magnitude))
		new_point = (closest_node.x + (normalized_direction[0] * step), closest_node.y + (normalized_direction[1] * step))
		if check_valid_node_rrt(new_point, obstacle_list, bounds):
			new_node = Node(new_point[0], new_point[1], 0, closest_node)
			nodes.append(new_node)
			current_node = new_node
	path_list = [(current_node.x, current_node.y)]
	path_node = current_node
	while path_node.parent_index != start_node:
		path_node = path_node.parent_index
		path_list.append((path_node.x, path_node.y))
	path_list.append((start_node.x, start_node.y))
	end_cost = 0
	for i in range(len(path_list)):
		if i != len(path_list) - 1:
			end_cost += distance(path_list[i], path_list[i + 1])

	return path_list, end_cost, nodes
