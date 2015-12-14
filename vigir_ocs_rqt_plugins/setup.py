#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_ocs_rqt_plugin_map_view_custom'],
    package_dir={'vigir_ocs_rqt_plugin_map_view_custom': 'vigir_ocs_rqt_plugin_map_view_custom/src/vigir_ocs_rqt_plugin_map_view_custom'}
)

setup(**d)

d = generate_distutils_setup(
    packages=['vigir_ocs_rqt_plugin_status_view_custom'],
    package_dir={'vigir_ocs_rqt_plugin_status_view_custom': 'vigir_ocs_rqt_plugin_status_view_custom/src/vigir_ocs_rqt_plugin_status_view_custom'}
)

setup(**d)
