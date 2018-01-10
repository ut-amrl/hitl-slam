#!/bin/sh

x-terminal-emulator -e 'bash -c "roscore"'
x-terminal-emulator -e 'bash -c "./localization_gui -m "EmptyMap""'
x-terminal-emulator -e 'bash -c "./HitL_SLAM -L "exampleData/Figure8/2016-02-16-16-01-46.bag.stfs.covars" -d1 -o && sleep 60s"'


