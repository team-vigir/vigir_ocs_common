#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_ocs_rqt_plugin_camera_viewer_custom'],
    package_dir={'vigir_ocs_rqt_plugin_camera_viewer_custom': 'vigir_ocs_rqt_plugin_camera_viewer_custom/src'}
)

setup(**d)

d = generate_distutils_setup(
    packages=['vigir_ocs_rqt_plugin_main_3d_view_custom'],
    package_dir={'vigir_ocs_rqt_plugin_main_3d_view_custom': 'vigir_ocs_rqt_plugin_main_3d_view_custom/src'}
)

setup(**d)

d = generate_distutils_setup(
    packages=['vigir_ocs_rqt_plugin_map_view_custom'],
    package_dir={'vigir_ocs_rqt_plugin_map_view_custom': 'vigir_ocs_rqt_plugin_map_view_custom/src'}
)

setup(**d)

d = generate_distutils_setup(
    packages=['vigir_ocs_rqt_plugin_status_view_custom'],
    package_dir={'vigir_ocs_rqt_plugin_status_view_custom': 'vigir_ocs_rqt_plugin_status_view_custom/src'}
)

setup(**d)
