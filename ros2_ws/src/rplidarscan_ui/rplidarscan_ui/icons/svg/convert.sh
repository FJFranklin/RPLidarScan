#!/bin/bash
for i in *.svg; do
  rsvg-convert -w 100 -h 100 -o ../png/${i%.svg}-100x100.png $i
  rsvg-convert -w  80 -h  80 -o ../png/${i%.svg}-80x80.png $i
done
