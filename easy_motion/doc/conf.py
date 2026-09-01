"""Sphinx configuration used by rosdoc2 for the easy_motion package."""

extensions = [
    'sphinx.ext.napoleon',
]

napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_use_param = True
napoleon_use_rtype = False

autodoc_typehints = 'signature'
autodoc_member_order = 'bysource'

# rosdoc2 also copies this Sphinx source directory under ``user_docs``.
# Exclude that duplicate while keeping the project files copied at the root.
exclude_patterns = ['user_docs*']

html_static_path = ['_static']
html_css_files = ['custom.css']
