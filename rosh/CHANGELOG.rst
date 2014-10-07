^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosh
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2014-10-07)
------------------
* super basic ability to run launch files
* add nodes property to LaunchableFile
* look for resources (nodes) in the right places
* Fix `#7 <https://github.com/OSUrobotics/rosh_core/issues/7>`_ (kill fails) - it looks like NodeNS._kill was never fully implemented
* Contributors: Dan Lazewatsky

1.0.7 (2014-08-17)
------------------
* replace rxgraph with rqt_graph (doesn't support ns args)
* Contributors: Dan Lazewatsky

1.0.6 (2014-07-25)
------------------
* fix a typo in Package._get_launches
* fixes from catkin_lint
* make sure rosh.impl gets installed
* Contributors: Dan Lazewatsky

1.0.5 (2014-07-07)
------------------
* fix errors caused by API changes
* Contributors: Dan Lazewatsky

1.0.4 (2014-06-26)
------------------
* warn if roscore isn't running (fixes `#1 <https://github.com/OSUrobotics/rosh_core/issues/1>`_)
* Fixing calls to deprecated roslib.rosenv (fixes `#2 <https://github.com/OSUrobotics/rosh_core/issues/2>`_)
* Contributors: Dan Lazewatsky

1.0.3 (2014-02-26)
------------------
* make sure shell.py gets put somewhere importable
* Contributors: Dan Lazewatsky

1.0.2 (2014-02-06)
------------------

1.0.1 (2014-02-06)
------------------
* Catkinized in preparation for release