#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##### BEGIN GPL LICENSE BLOCK #####
#  
# Copyright (C) 2017  Patrick Baus
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# ##### END GPL LICENSE BLOCK #####

import argparse
import fnmatch
import os
import re
import numpy

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Parser for R&S FPC1500 csv files')
    parser.add_argument("filter", help="Filter used to specify the files on which to run the parser (e.g. \"*.csv\").")
    args = parser.parse_args()
    directory = "./"

    for filename in os.listdir(directory):
        if fnmatch.fnmatch(filename, args.filter):
            print("Parsing file {filename!s}".format(filename=filename))
            # String looks like this:
            # >>> print(repr(line))
            # 'NumberPoints,64001\n'
            #regex_number_of_points = re.compile("^NumberPoints,(\d+)\n$")
            # String looks like this:
            # >>> print(repr(line))
            # 'Center Frequency,50002500,Hz,,\n'
            regex_center = re.compile("^Center Frequency,([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?),([A-Za-z]+),,\n$")
            # String looks like this:
            # >>> print(repr(line))
            # 'Span,99995000,Hz,,'
            regex_span = re.compile("^Span,([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?),([A-Za-z]+),,\n$")
            # String looks like this:
            # >>> print(repr(line))
            # '5000,-21.1552410125732,,,'
            regex_value = re.compile("^([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?),([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?),([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?),,\n$")

            lines = open(directory + filename, "r")

            # create a new directy for the parsed files
            path = "./parsed/"
            if not os.path.exists(path):
                os.makedirs(path)
            # split the filename and the extension to rename the file to a ".csv" extension
            fn, fe = os.path.splitext(filename)
            output_file = open(path + fn + ".csv", "w")

            center_point = None
            span = None
            unit = None
            values = []

            for line in lines:
              if center_point is None and regex_center.match(line):
                center_point = float(regex_center.match(line).group(1))
                unit = regex_center.match(line).group(2)
                print("Center point: {value} {unit}".format(value=center_point, unit=unit))
              if span is None and regex_span.match(line):
                span = float(regex_span.match(line).group(1))
                print("Span: {value} {unit}".format(value=span, unit=unit))
             
              if center_point is not None and span is not None and regex_value.match(line):
                  values.append(float(regex_value.match(line).group(2)))
                
            print("{number_of_values} values read".format(number_of_values=len(values)))

            # close all open file handles
            lines.close()
            if len(values) > 0:
              data = numpy.column_stack((numpy.linspace(center_point-span/2, center_point+span/2, num=len(values)), values))
              numpy.savetxt(path + fn + ".csv", data, fmt=["%.5f", "%.15f"], delimiter=",")
