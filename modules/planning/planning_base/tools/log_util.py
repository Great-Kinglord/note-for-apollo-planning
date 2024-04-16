#!/usr/bin/env python3

###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

# -*- coding:utf-8 -*-
import sys
import re
import matplotlib.pyplot as plt


def get_string_between(string, st, ed=''):
    """get string between keywords"""
    if string.find(st) < 0:
        return ''
    sub_string = string[string.find(st) + len(st):]
    if len(ed) == 0 or sub_string.find(ed) < 0:
        return sub_string.strip()
    return sub_string[:sub_string.find(ed)]


def get_planning_seq_num(line):
    """get planning seq num from line"""
    return get_string_between(line, 'start frame sequence id = [', ']')


def get_time(line):
    """get time from line"""
    return get_string_between(line, ' ', ' ')


def get_lines_between(lines, st, ed=''):
    """get valid log with keywords"""
    valid_lines = []
    found_start = False
    for line in lines:
        if st in line:
            found_start = True
        if len(ed) != 0 and ed in line:
            break
        if found_start:
            valid_lines.append(line)
    return valid_lines


def search_next(lines, line_search_num):
    """search forward, return frame start and end line number"""
    start_line_num = -1
    seq_id = '-1'
    for i in range(line_search_num, len(lines)):
        if 'Planning start frame sequence id' in lines[i]:
            seq_id = get_string_between(lines[i], 'sequence id = [', ']')
            start_line_num = i
            break
    if start_line_num < 0:
        return -1, -1, seq_id

    for i in range(start_line_num, len(lines)):
        if 'Planning end frame sequence id = [' + seq_id in lines[i]:
            return start_line_num, i, seq_id
    return start_line_num, -1, seq_id


def search_current(lines, line_search_num):
    """search current frame, return frame start and end line number"""
    start_line_num = -1
    end_line_num = -1
    seq_id = '-1'
    for i in range(line_search_num, 0, -1):
        if 'Planning start frame sequence id' in lines[i]:
            seq_id = get_string_between(lines[i], 'sequence id = [', ']')
            start_line_num = i
            break
    for i in range(line_search_num, len(lines) - 1):
        if 'Planning end frame sequence id = [' + seq_id in lines[i]:
            end_line_num = i

    return start_line_num, end_line_num, seq_id


def search_last(lines, line_search_num):
    end_line_num = -1
    seq_id = '-1'
    for i in range(line_search_num, 0, -1):
        if 'Planning end frame sequence id' in lines[i]:
            seq_id = get_string_between(lines[i], 'sequence id = [', ']')
            end_line_num = i
            break
    if end_line_num < 0:
        return -1, -1, seq_id

    for i in range(end_line_num, 0, -1):
        if 'Planning start frame sequence id = [' + seq_id in lines[i]:
            return i, end_line_num, seq_id
    return -1, end_line_num, seq_id


def search_time_line(lines, search_time):
    """search line with time"""
    for i in range(len(lines)):
        if search_time in lines[i]:
            return i + 1
    return 0


def search_keyword_line(lines, keyword):
    """search line with time"""
    for i in range(len(lines)):
        if keyword in lines[i]:
            return i
    return 0


def get_points_from_line(line, data):
    pat = re.compile(r'[(](.*?)[)]', re.S)
    str_list = re.findall(pat, line)
    for string in str_list:
        num = string.split(",")
        data.append([float(num[0]), float(num[1])])


def get_data_from_line(line, data):
    pat = re.compile(r'[(](.*?)[)]', re.S)
    str_list = re.findall(pat, line)
    for string in str_list:
        num = string.split(",")
        data[0].append(float(num[0]))
        data[1].append(float(num[1]))


class MouseEventManager(object):
    x, y = 0.0, 0.0
    xoffset, yoffset = -20, 20
    text_template = 'x: %0.2f\ny: %0.2f'
    annotation = False

    def on_click(self, event):
        # if mouse button is not right, return
        # 1: left, 2: middle, 3: right
        if event.button is not 3:
            return
        self.x, self.y = event.xdata, event.ydata
        if self.x is not None:
            print('mouse click x: %.2f, y: %.2f' % (event.xdata, event.ydata))
            if self.annotation:
                self.annotation.set_visible(False)
            label_text = self.text_template % (self.x, self.y)
            self.annotation = event.inaxes.annotate(label_text,
                                                    xy=(self.x, self.y), xytext=(
                                                        self.xoffset, self.yoffset),
                                                    textcoords='offset points', ha='right', va='bottom',
                                                    bbox=dict(
                                                        boxstyle='round,pad=0.5', fc='lightcyan', alpha=0.5),
                                                    arrowprops=dict(
                                                        arrowstyle='->', connectionstyle='arc3,rad=0')
                                                    )
            self.annotation.set_visible(True)
            self.annotation.figure.canvas.draw()


class Index(object):
    """button callback function"""

    def __init__(self, fig, ax, line_st_num, line_ed_num, lines, config):
        self.ax = ax
        self.fig = fig
        self.line_st_num = line_st_num
        self.line_ed_num = line_ed_num
        self.lines = lines
        self.config = config
        self.line_map = {}
        for line in config["line"]:
            self.line_map[line["label"]] = line
        self.reset_mouse_event()

    def reset_mouse_event(self):
        self.mouse_manager = MouseEventManager()
        self.fig.canvas.mpl_connect('button_release_event',
                                    self.mouse_manager.on_click)

    def next(self, step):
        """next button callback function"""
        for i in range(step):
            line_st_num, line_ed_num, seq_id = search_next(
                self.lines, self.line_ed_num + 1)
            # check whether found frame log is complete
            if line_st_num < 0 or line_ed_num < 0:
                print('[ERROR] search reach last line, may reach last frame')
                return
            self.line_st_num = line_st_num
            self.line_ed_num = line_ed_num
        self.plot_frame("frame: " + seq_id)
        self.reset_mouse_event()

    def next1(self, event):
        self.next(1)

    def next10(self, event):
        self.next(10)

    def prev(self, step):
        """prev button callback function"""
        for i in range(step):
            line_st_num, line_ed_num, seq_id = search_last(
                self.lines, self.line_st_num - 1)
            # check whether found frame log is complete
            if line_st_num < 0 or line_ed_num < 0:
                print('[ERROR] search reach first line, may reach first frame')
                return
            self.line_st_num = line_st_num
            self.line_ed_num = line_ed_num
        self.plot_frame("frame: " + seq_id)
        self.reset_mouse_event()

    def prev1(self, event):
        self.prev(1)

    def prev10(self, event):
        self.prev(10)

    def exit(self, event):
        sys.exit(0)

    def plot_frame(self, fig_name):
        print('plot line start num: ' + str(self.line_st_num + 1))
        print('plot line end num: ' + str(self.line_ed_num + 1))
        data = {}
        self.graphs = {}
        self.fig.suptitle(fig_name)
        for item in self.config["subplot"]:
            key = item["title"]
            self.ax[key].clear()
            self.ax[key].set_xlabel(item["x_label"])
            self.ax[key].set_ylabel(item["y_label"])
            self.ax[key].set_title(item["title"])
        for i in range(self.line_st_num, self.line_ed_num):
            line = self.lines[i]
            for key in self.line_map:
                if key.find("print_") == -1:
                    name = "] print_" + key + ":"
                else:
                    name = "] " + key + ":"
                matched = re.findall(name, line)
                if len(matched) > 0:
                    origin_key = matched[0].strip(
                        "] print").strip("_").strip(":")
                    data[origin_key] = ([], [], key)
                    print("key", key)
                    print("matched[0]", matched[0])
                    print("name", name)
                    print("origin_key", origin_key)
                    get_data_from_line(line, data[origin_key])
        for name in data:
            print("name", name)
            key = data[name][2]
            subplot_name = self.line_map[key]["subplot"]
            if not subplot_name in self.ax:
                continue
            if len(data[name]) > 0:
                line, = self.ax[subplot_name].plot(
                    data[name][0], data[name][1], self.line_map[key]["marker"], label=name)
                self.ax[subplot_name].grid(True)
                self.graphs[name] = line
        for item in self.config["subplot"]:
            key = item["title"]
            legend = self.ax[key].legend()
            for legend_line in legend.get_lines():
                legend_line.set_picker(True)
                legend_line.set_pickradius(10)
            # self.ax[key].set_aspect(1)
        plt.connect('pick_event', self.on_pick)
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
        plt.draw()
        return

    def on_pick(self, event):

        legend = event.artist
        isVisible = legend.get_visible()
        self.graphs[legend._label].set_visible(not isVisible)
        legend.set_visible(not isVisible)
        self.fig.canvas.draw()
