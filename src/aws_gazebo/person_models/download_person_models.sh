#!/bin/sh

git clone https://bitbucket.org/theconstructcore/person_sim.git

# unzip & remove
mv person_sim/person_sim/models .
rm -r person_sim