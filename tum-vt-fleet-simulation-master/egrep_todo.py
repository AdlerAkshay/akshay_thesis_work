#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# ########################################################################### #
"""################################
@author: Florian Dandl         #
@contact: florian.dandl@tum.de #
################################
"""
# ########################################################################### #
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# general imports
import os
import sys
import re
#
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# script doc
scriptname = os.path.basename(__file__)
__doc__ += """
# {1} # 
# {0} #
# {1} #

Description:
------------
This script searches for a regular expression in a list of files.
The egrep functionality is general, the old_main-function is special for this case.
The script searches automatically in all sub-directories.

Input parameters:
-----------------
- regex_str

Output:
-------
- last_egrep_code_results.txt

Call:
-----
python {0} regex_str
""".format(scriptname, "-"*len(scriptname))
#
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# code import
#
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# global variables
#
script_dir = os.path.dirname(__file__)
#
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# help functions
#
def logprint(prt_str, fhout):
    # the first part only works if fhout exists
    try:
        fhout.write(prt_str + "\n")
    except:
        pass
    print(prt_str)
#
def egrep(regex_str, list_f, output_f=None):
    code_line_counter = 0
    empty_line_counter = 0
    comment_line_counter = 0
    number_matches = 0
    print("Regex: {}".format(regex_str))
    print("Searching in {} files: {}".format(len(list_f), list_f))
    print("Output file: {}".format(output_f))
    fhout = None
    if output_f:
        fhout = open(output_f, 'w')
    #
    regex = re.compile(regex_str)
    #
    for f in list_f:
        logprint("Searching in {}".format(f), fhout)
        fhin = open(f, 'r')
        line_counter = 0
        for line in fhin:
            line_counter += 1
            if not line.strip():
                empty_line_counter += 1
            elif line.strip().startswith("#"):
                comment_line_counter += 1
            else:
                code_line_counter += 1
            match = regex.search(line.strip())
            if match:
                number_matches += 1
                logprint("... line {}: {}".format(line_counter, line.strip()), fhout)
        fhin.close()
    logprint("#"*80, fhout)
    logprint("Number of code lines: {}".format(code_line_counter), fhout)
    logprint("Number of comment lines: {}".format(comment_line_counter), fhout)
    logprint("Number of empty lines: {}".format(empty_line_counter), fhout)
    logprint("Number of matches for {}: {}".format(regex_str, number_matches), fhout)
    if output_f:
        fhout.close()
#
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# class definitions
#
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# old_main functions
#
def main(regex_str):
    output_f = 'last_egrep_todo_results.log'
    list_f = []
    top_dir = "src"
    for root, dirs, files in os.walk(top_dir):
        for f in files:
            if f.endswith(".py"):
                list_f.append(os.path.join(root, f))
    #
    #print(list_f)
    egrep(regex_str, list_f, output_f)
#
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# script call
if __name__ == '__main__':
    # print(__doc__)
    # if len(sys.argv) == 2:
    #     main(sys.argv[1])
    regex = "# TODO #"
    main(regex)

