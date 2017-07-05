from __future__ import print_function

# Based on source code Developed by: andreasBihlmaier
# source:   https://github.com/andreasBihlmaier/gazebo2rviz
#           https://github.com/andreasBihlmaier/pysdf

import re


def sdf2tfname(sdfname):
    return sdfname.replace('::', '__').replace('@', 'AT')


def name2modelname(name):
    # Cope with
    # - adding the same model multiple times through the GUI
    # - renamed models (e.g. because model occurs multiple times as sub-model)
    modelname = re.sub('_[0-9]*$', '', name)
    modelname = re.sub('@.*$', '', modelname)
    return modelname
