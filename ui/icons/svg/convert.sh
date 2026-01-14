#!/bin/bash
for i in *.svg; do
  rsvg-convert -w 100 -h 100 -o ../png/${i%.svg}-100x100.png $i
done
